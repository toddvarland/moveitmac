import Darwin
import Foundation

// MARK: - myCobot-280 Hardware Bridge
//
// Communicates with an Elephant Robotics myCobot-280 M5 over USB serial
// using the robot's binary frame protocol.
//
// Frame format (18 bytes for SET_ANGLES):
//   [0xFE, 0xFE, LEN, CMD, J1H, J1L, …, J6H, J6L, SPEED, 0xFA]
//   LEN = 1 (CMD byte) + 12 (6 × int16 angles) + 1 (speed) = 0x0E
//   Angles are in units of 0.01 degrees, big-endian two's complement int16.
//
// Thread safety: all methods must be called on the main actor.

@MainActor
final class MyCobotBridge: ObservableObject {

    // MARK: - Published state

    @Published private(set) var isConnected   = false
    @Published private(set) var connectedPort = ""
    @Published private(set) var lastError: String? = nil

    // MARK: - Private

    private var fd: Int32 = -1
    /// All writes are dispatched onto this queue so blocking I/O never stalls the main thread.
    private let writeQueue = DispatchQueue(label: "mycobot.serial", qos: .userInteractive)
    /// GET_ANGLES requests and their reads run on this queue, separate from writes.
    private let readQueue  = DispatchQueue(label: "mycobot.serial.read", qos: .utility)

    // MARK: - Port discovery

    /// Returns a sorted list of candidate USB serial ports in /dev/.
    static func availablePorts() -> [String] {
        guard let entries = try? FileManager.default
                .contentsOfDirectory(atPath: "/dev") else { return [] }
        return entries
            .filter { $0.hasPrefix("cu.") }
            .sorted()
            .map { "/dev/\($0)" }
    }

    // MARK: - Connect / Disconnect

    /// Open the serial port and configure it for 115200 8N1 communication.
    func connect(to port: String) {
        disconnect()
        lastError = nil

        let newFd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC)
        guard newFd >= 0 else {
            lastError = "Cannot open \(port): \(String(cString: strerror(errno)))"
            return
        }
        // Clear O_NONBLOCK so writes block until the UART drains.
        // O_NONBLOCK is needed during open() to avoid hanging on a modem,
        // but must be removed afterwards or write() silently drops bytes.
        let flags = fcntl(newFd, F_GETFL)
        if flags >= 0 { _ = fcntl(newFd, F_SETFL, flags & ~O_NONBLOCK) }

        var tty = termios()
        guard tcgetattr(newFd, &tty) == 0 else {
            close(newFd)
            lastError = "tcgetattr failed"
            return
        }

        cfmakeraw(&tty)
        cfsetspeed(&tty, speed_t(B115200))
        tty.c_cflag &= ~tcflag_t(CSIZE)
        tty.c_cflag |= tcflag_t(CS8)
        tty.c_cflag &= ~tcflag_t(PARENB)
        tty.c_cflag &= ~tcflag_t(CSTOPB)
        tty.c_cflag |= tcflag_t(CREAD) | tcflag_t(CLOCAL)
        // VMIN=0, VTIME=2: read() times out after 200 ms instead of blocking forever.
        // This lets pollAngles recover gracefully when the robot doesn't respond.
        withUnsafeMutableBytes(of: &tty.c_cc) { cc in
            cc[Int(VMIN)]  = 0
            cc[Int(VTIME)] = 2   // units of 100 ms → 200 ms timeout
        }

        guard tcsetattr(newFd, TCSANOW, &tty) == 0 else {
            close(newFd)
            lastError = "tcsetattr failed"
            return
        }

        fd            = newFd
        connectedPort = port
        isConnected   = true
    }

    func disconnect() {
        guard fd >= 0 else { return }
        close(fd)
        fd            = -1
        isConnected   = false
        connectedPort = ""
    }

    // MARK: - Send angles

    // Joint sign corrections: some joints have inverted angle convention between
    // the URDF coordinate frame and the myCobot 280-M5 firmware.
    // +1 = same sign, -1 = inverted.  Determined empirically by jogging each joint
    // and comparing virtual-arm direction with physical-arm direction.
    // J4 axis is defined as 0 0 -1 in the URDF, so no additional bridge flip needed.
    private static let jointSignCorrection: [Double] = [1, 1, 1, 1, 1, 1]

    /// Send a single joint to a target angle (radians) at the given speed.
    func sendAngle(jointIndex: Int, radians: Double, robotSpeed: Int) {
        guard isConnected else { return }
        let spd = UInt8(robotSpeed.clamped(to: 1 ... 100))
        let sign = jointIndex < Self.jointSignCorrection.count ? Self.jointSignCorrection[jointIndex] : 1.0
        let deg = radians * sign * (180.0 / .pi)
        let raw = Int((deg * 100).rounded()).clamped(to: Int(Int16.min) ... Int(Int16.max))
        let encoded = raw >= 0 ? raw : raw + 65536
        let params: [UInt8] = [UInt8(jointIndex + 1), UInt8(encoded >> 8), UInt8(encoded & 0xFF), spd]
        enqueueFrame(cmd: 0x21, params: params)
    }

    /// Send all 6 joints using individual SET_ANGLE frames with 50 ms gaps.
    /// Runs on a fresh Task so it's never blocked by queued jog frames.
    func sendAngles(_ radians: [Double], robotSpeed: Int) {
        guard isConnected, fd >= 0, radians.count >= 6 else {
            print("[Bridge] sendAngles skipped — isConnected=\(isConnected) fd=\(fd)")
            return
        }
        print("[Bridge] sendAngles called fd=\(fd)")
        let spd = UInt8(robotSpeed.clamped(to: 1 ... 100))
        var frames: [[UInt8]] = []
        for i in 0 ..< 6 {
            let sign = i < Self.jointSignCorrection.count ? Self.jointSignCorrection[i] : 1.0
            let deg = radians[i] * sign * (180.0 / .pi)
            let raw = Int((deg * 100).rounded()).clamped(to: Int(Int16.min) ... Int(Int16.max))
            let encoded = raw >= 0 ? raw : raw + 65536
            frames.append(buildFrame(cmd: 0x21, params: [UInt8(i + 1), UInt8(encoded >> 8), UInt8(encoded & 0xFF), spd]))
        }
        let capturedFd = fd
        Task.detached(priority: .userInitiated) {
            for (i, frame) in frames.enumerated() {
                var f = frame
                let n = Darwin.write(capturedFd, &f, f.count)
                print("[Bridge] frame \(i+1)/6 wrote \(n) bytes: \(f)")
                usleep(50_000)
            }
        }
    }

    // MARK: - Read angles (GET_ANGLES, CMD 0x20)

    /// Send a GET_ANGLES request and return 6 joint angles (radians) via callback on the main queue.
    /// Fires `completion(nil)` on timeout or parse failure.  Safe to call only when the
    /// servo engine is idle (no concurrent jog writes on the wire).
    func pollAngles(completion: @escaping ([Double]?) -> Void) {
        guard isConnected, fd >= 0 else { return }
        let capturedFd = fd
        // GET_ANGLES request: [0xFE, 0xFE, 0x02, 0x20, 0xFA]
        var req: [UInt8] = [0xFE, 0xFE, 0x02, 0x20, 0xFA]
        readQueue.async {
            // Flush stale input so we don't accidentally parse a leftover frame.
            tcflush(capturedFd, TCIFLUSH)
            _ = Darwin.write(capturedFd, &req, req.count)
            // Give the robot ~80 ms to formulate and transmit its 17-byte response.
            usleep(80_000)
            // Read up to 64 bytes; VTIME=2 provides a 200 ms backstop if data is late.
            var buf = [UInt8](repeating: 0, count: 64)
            var totalRead = 0
            for _ in 0 ..< 4 {
                let n = buf.withUnsafeMutableBytes { ptr in
                    Darwin.read(capturedFd, ptr.baseAddress!.advanced(by: totalRead),
                                64 - totalRead)
                }
                guard n > 0 else { break }
                totalRead += n
                if totalRead >= 17 { break }
            }
            guard totalRead >= 17 else {
                DispatchQueue.main.async { completion(nil) }
                return
            }
            let data = Array(buf.prefix(totalRead))
            // Scan for a valid response frame:
            // [0xFE, 0xFE, LEN, 0x20, J1H, J1L, …, J6H, J6L, 0xFA]  (17 bytes)
            for i in 0 ... (data.count - 17) {
                guard data[i] == 0xFE, data[i + 1] == 0xFE,
                      data[i + 3] == 0x20, data[i + 16] == 0xFA else { continue }
                var angles = [Double]()
                for j in 0 ..< 6 {
                    let hi  = UInt16(data[i + 4 + j * 2])
                    let lo  = UInt16(data[i + 5 + j * 2])
                    let raw = Int16(bitPattern: (hi << 8) | lo)
                    // Protocol units: 0.01 °; convert to radians for URDF.
                    // Apply sign correction to match URDF joint convention.
                    let sign = j < MyCobotBridge.jointSignCorrection.count ? MyCobotBridge.jointSignCorrection[j] : 1.0
                    angles.append(Double(raw) / 100.0 * .pi / 180.0 * sign)
                }
                DispatchQueue.main.async { completion(angles) }
                return
            }
            DispatchQueue.main.async { completion(nil) }
        }
    }

    // MARK: - Frame helpers

    private func buildFrame(cmd: UInt8, params: [UInt8]) -> [UInt8] {
        var frame: [UInt8] = [0xFE, 0xFE, UInt8(params.count + 2), cmd]
        frame.append(contentsOf: params)
        frame.append(0xFA)
        return frame
    }

    private func enqueueFrame(cmd: UInt8, params: [UInt8]) {
        guard fd >= 0 else { return }
        var frame = buildFrame(cmd: cmd, params: params)
        let capturedFd = fd
        writeQueue.async {
            _ = Darwin.write(capturedFd, &frame, frame.count)
        }
    }

    deinit {
        if fd >= 0 { close(fd) }
    }
}

// MARK: - Comparable clamping helper (local, avoids polluting global namespace)

private extension Comparable {
    func clamped(to range: ClosedRange<Self>) -> Self {
        min(max(self, range.lowerBound), range.upperBound)
    }
}
