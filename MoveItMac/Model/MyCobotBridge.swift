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

    /// Transmit a SET_ANGLES frame to the robot.
    ///
    /// - Parameters:
    ///   - radians: Joint angles in radians, BFS order J1–J6 (first 6 used).
    ///   - robotSpeed: Firmware speed parameter 1–100.
    func sendAngles(_ radians: [Double], robotSpeed: Int) {
        guard isConnected, fd >= 0, radians.count >= 6 else { return }

        // Fixed 18-byte frame.
        var frame = [UInt8](repeating: 0, count: 18)
        frame[0] = 0xFE
        frame[1] = 0xFE
        frame[2] = 0x0E           // LEN = 14
        frame[3] = 0x20           // SET_ANGLES command

        for i in 0 ..< 6 {
            let deg = radians[i] * (180.0 / .pi)
            let raw = Int((deg * 100).rounded())
                .clamped(to: Int(Int16.min) ... Int(Int16.max))
            // Encode as big-endian two's complement int16.
            let encoded = raw >= 0 ? raw : raw + 65536
            frame[4 + i * 2]     = UInt8(encoded >> 8)
            frame[4 + i * 2 + 1] = UInt8(encoded & 0xFF)
        }

        frame[16] = UInt8(robotSpeed.clamped(to: 1 ... 100))
        frame[17] = 0xFA

        // Non-blocking write — EAGAIN (kernel buffer full) silently drops the frame.
        frame.withUnsafeBytes { ptr in
            _ = Darwin.write(fd, ptr.baseAddress!, frame.count)
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
