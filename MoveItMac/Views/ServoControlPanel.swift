import SwiftUI
import URDFKit

// MARK: - Servo Control Panel

/// Non-blocking popover panel for real-time servo jogging.
/// Displayed as a toolbar popover so the 3-D viewport remains visible.
struct ServoControlPanel: View {
    @EnvironmentObject var appState: AppState
    @ObservedObject var servo: ServoEngine

    @State private var availablePorts:    [String] = []
    @State private var selectedPort:      String   = ""
    @State private var isCapturingCenter: Bool     = false

    /// 5 Hz clock used to read back physical joint angles when idle.
    private let feedbackClock = Timer.publish(every: 0.2, on: .main, in: .common).autoconnect()

    var body: some View {
        VStack(spacing: 0) {
            header
            Divider()
            Group {
                switch servo.mode {
                case .joint:     jointControls
                case .cartesian: cartesianControls
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            Divider()
            footer
        }
        .frame(width: 340, height: 540)
        .onAppear  { rescanPorts() }
        .onDisappear { servo.clearCommands() }
        .onReceive(feedbackClock) { _ in
            // Mirror physical arm → virtual model when idle and no planning session is active.
            // Suspend polling once the user has set a start/goal so that manually positioning
            // the virtual arm (via sliders) isn't overwritten by readback every 200 ms.
            guard servo.bridge.isConnected,
                  servo.status == .idle,
                  appState.planStart.isEmpty,
                  appState.plannerStatus == .idle else { return }
            servo.bridge.pollAngles { angles in
                guard let angles, let model = appState.robotModel else { return }
                let names = model.orderedActuatedJointNames
                for (i, name) in names.prefix(6).enumerated() {
                    appState.jointAngles[name] = angles[i]
                }
                servo.angleRevision += 1
            }
        }
    }

    // MARK: - Header

    private var header: some View {
        VStack(spacing: 0) {
            // Status + mode row
            HStack(spacing: 12) {
                statusBadge
                Spacer()
                Picker("Mode", selection: $servo.mode) {
                    ForEach(ServoEngine.Mode.allCases, id: \.self) { m in
                        Text(m.rawValue).tag(m)
                    }
                }
                .pickerStyle(.segmented)
                .frame(width: 160)
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 10)

            Divider()
            hardwareRow
            Divider()
            poseActionsRow
        }
        .background(.bar)
    }

    // MARK: - Pose Actions Row (Zero / Center)

    private var poseActionsRow: some View {
        VStack(spacing: 0) {
            // Row 1: Zero Pose + Go to Center
            HStack(spacing: 8) {
                Button {
                    servo.stop()
                    if let model = appState.robotModel {
                        for name in model.orderedActuatedJointNames {
                            appState.jointAngles[name] = 0
                        }
                        servo.angleRevision += 1
                    }
                    if servo.bridge.isConnected {
                        servo.bridge.sendAngles([0, 0, 0, 0, 0, 0], robotSpeed: 50)
                    }
                } label: {
                    Label("Zero Pose", systemImage: "arrow.up.to.line")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)

                Button {
                    guard let center = appState.centerAngles,
                          let model  = appState.robotModel else { return }
                    servo.stop()
                    appState.jointAngles = center
                    servo.angleRevision += 1
                    if servo.bridge.isConnected {
                        let names  = model.orderedActuatedJointNames
                        let angles = Array(names.prefix(6).map { center[$0] ?? 0 })
                        servo.bridge.sendAngles(angles, robotSpeed: 50)
                    }
                } label: {
                    Label("Go to Center", systemImage: "scope")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                .disabled(appState.centerAngles == nil)
            }
            .padding(.horizontal, 14)
            .padding(.top, 6)

            // Row 2: Capture Center from Arm (only when connected + idle)
            if servo.bridge.isConnected {
                Button {
                    guard !isCapturingCenter else { return }
                    isCapturingCenter = true
                    servo.bridge.pollAngles { angles in
                        isCapturingCenter = false
                        guard let angles, let model = appState.robotModel else { return }
                        let names = model.orderedActuatedJointNames
                        var captured: [String: Double] = [:]
                        for (i, name) in names.prefix(6).enumerated() {
                            captured[name] = angles[i]
                        }
                        appState.centerAngles = captured
                        // Sync virtual arm to the captured pose.
                        appState.jointAngles = captured
                        servo.angleRevision += 1
                    }
                } label: {
                    if isCapturingCenter {
                        Label("Reading\u{2026}", systemImage: "arrow.down.circle")
                            .frame(maxWidth: .infinity)
                    } else {
                        Label(
                            appState.centerAngles == nil
                                ? "Capture Center from Arm"
                                : "Re-capture Center from Arm",
                            systemImage: "arrow.down.circle"
                        )
                        .frame(maxWidth: .infinity)
                    }
                }
                .buttonStyle(.bordered)
                .tint(.orange)
                .disabled(isCapturingCenter || servo.status != .idle)
                .padding(.horizontal, 14)
                .padding(.top, 4)
            }
        }
        .padding(.bottom, 6)
        .background(.bar)
    }

    // MARK: - Hardware Row

    private var hardwareRow: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(spacing: 8) {
                Image(systemName: "cable.connector")
                    .foregroundStyle(servo.bridge.isConnected ? .green : .secondary)
                    .frame(width: 16)

                Picker("Port", selection: $selectedPort) {
                    Text("Select port…").tag("")
                    ForEach(availablePorts, id: \.self) { port in
                        Text(port.replacingOccurrences(of: "/dev/", with: "")).tag(port)
                    }
                }
                .labelsHidden()
                .frame(maxWidth: .infinity)

                Button { rescanPorts() } label: {
                    Image(systemName: "arrow.clockwise")
                }
                .buttonStyle(.plain)
                .foregroundStyle(.secondary)

                Button(servo.bridge.isConnected ? "Disconnect" : "Connect") {
                    if servo.bridge.isConnected {
                        servo.bridge.disconnect()
                    } else if !selectedPort.isEmpty {
                        servo.bridge.connect(to: selectedPort)
                    }
                }
                .disabled(!servo.bridge.isConnected && selectedPort.isEmpty)
                .buttonStyle(.bordered)
                .tint(servo.bridge.isConnected ? .red : nil)
            }
            if let err = servo.bridge.lastError {
                Text(err)
                    .font(.caption)
                    .foregroundStyle(.red)
                    .lineLimit(1)
            }
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 8)
    }

    private func rescanPorts() {
        availablePorts = MyCobotBridge.availablePorts()
        if selectedPort.isEmpty || !availablePorts.contains(selectedPort) {
            // Prefer known myCobot USB-serial chip names (CP2104 / SLAB / FTDI)
            selectedPort = availablePorts.first(where: {
                $0.contains("usbserial") || $0.contains("SLAB") || $0.contains("usbmodem")
            }) ?? availablePorts.first ?? ""
        }
    }

    private var statusBadge: some View {
        HStack(spacing: 5) {
            Circle().fill(statusColor).frame(width: 8, height: 8)
            Text(statusLabel)
                .font(.system(size: 11, weight: .semibold))
                .foregroundStyle(statusColor)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 3)
        .background(statusColor.opacity(0.12), in: Capsule())
    }

    private var statusColor: Color {
        switch servo.status {
        case .idle:    return .secondary
        case .running: return .green
        case .halted:  return .red
        }
    }

    private var statusLabel: String {
        switch servo.status {
        case .idle:    return "Idle"
        case .running: return "Running"
        case .halted:  return "Halted — Collision"
        }
    }

    // MARK: - Footer

    private var footer: some View {
        HStack(spacing: 12) {
            VStack(alignment: .leading, spacing: 2) {
                Text("Speed  \(String(format: "%.1f×", servo.speedScale))")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Slider(value: $servo.speedScale, in: 0.1...2.0)
                    .frame(width: 130)
            }
            Spacer()
            actionButtons
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 10)
        .background(.bar)
    }

    @ViewBuilder
    private var actionButtons: some View {
        switch servo.status {
        case .idle:
            Button("Start") { servo.start(appState: appState) }
                .buttonStyle(.borderedProminent)
                .disabled(!appState.isRobotLoaded)
        case .running:
            Button("Stop", role: .destructive) { servo.stop() }
                .buttonStyle(.bordered)
        case .halted:
            HStack(spacing: 8) {
                Button("Resume") { servo.resume() }
                    .buttonStyle(.bordered)
                Button("Stop", role: .destructive) { servo.stop() }
                    .buttonStyle(.bordered)
            }
        }
    }

    // MARK: - Joint Controls

    private var jointControls: some View {
        Group {
            if let model = appState.robotModel {
                let names = model.orderedActuatedJointNames
                // servo.angleRevision is @Published and increments at ~20 Hz,
                // driving re-renders of this view so angle text stays current.
                let _ = servo.angleRevision
                ScrollView {
                    VStack(spacing: 0) {
                        ForEach(names, id: \.self) { name in
                            JointJogRow(
                                name:      name,
                                angleDeg:  (appState.jointAngles[name] ?? 0) * 180 / .pi,
                                isRunning: servo.status == .running,
                                onPress:   { dir in servo.setJoint(name, direction: dir) },
                                onRelease: { servo.setJoint(name, direction: 0) }
                            )
                            Divider().padding(.leading, 8)
                        }
                    }
                    .padding(.vertical, 4)
                    .frame(maxWidth: .infinity)
                }
            } else {
                ContentUnavailableView("No Robot Loaded", systemImage: "gearshape.2")
            }
        }
    }

    // MARK: - Cartesian Controls

    private var cartesianControls: some View {
        VStack(spacing: 24) {
            Spacer()

            VStack(spacing: 8) {
                Text("Translation (World Frame)")
                    .font(.caption).foregroundStyle(.secondary)
                HStack(spacing: 16) {
                    CartesianJogPair(label: "X", labelColor: .red,   axisIndex: 0, servo: servo)
                    CartesianJogPair(label: "Y", labelColor: .green, axisIndex: 1, servo: servo)
                    CartesianJogPair(label: "Z", labelColor: .blue,  axisIndex: 2, servo: servo)
                }
            }

            VStack(spacing: 8) {
                Text("Rotation (World Frame)")
                    .font(.caption).foregroundStyle(.secondary)
                HStack(spacing: 16) {
                    CartesianJogPair(label: "Rx", labelColor: .red,   axisIndex: 3, servo: servo)
                    CartesianJogPair(label: "Ry", labelColor: .green, axisIndex: 4, servo: servo)
                    CartesianJogPair(label: "Rz", labelColor: .blue,  axisIndex: 5, servo: servo)
                }
            }

            if appState.robotSetup.activeGroup == nil {
                Label(
                    "Set an active planning group in Setup Assistant for accurate Cartesian jogging.",
                    systemImage: "info.circle"
                )
                .font(.caption)
                .foregroundStyle(.orange)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 14)
            }

            Spacer()
        }
        .padding(.horizontal, 14)
    }
}

// MARK: - Joint Jog Row

private struct JointJogRow: View {
    let name:      String
    let angleDeg:  Double
    let isRunning: Bool
    let onPress:   (Double) -> Void
    let onRelease: () -> Void

    var body: some View {
        HStack(spacing: 6) {
            Text(name)
                .font(.system(size: 11, design: .monospaced))
                .lineLimit(1)
                .frame(maxWidth: .infinity, alignment: .leading)
            Text(String(format: "%.1f°", angleDeg))
                .font(.system(size: 11, design: .monospaced))
                .foregroundStyle(.secondary)
                .frame(width: 52, alignment: .trailing)
            HoldButton(systemImage: "minus") { onPress(-1) } onRelease: { onRelease() }
                .disabled(!isRunning)
            HoldButton(systemImage: "plus")  { onPress(+1) } onRelease: { onRelease() }
                .disabled(!isRunning)
        }
        .padding(.vertical, 1)
    }
}

// MARK: - Cartesian Jog Pair

private struct CartesianJogPair: View {
    let label:      String
    let labelColor: Color
    let axisIndex:  Int
    @ObservedObject var servo: ServoEngine

    private var isRunning: Bool { servo.status == .running }

    var body: some View {
        VStack(spacing: 4) {
            Text(label)
                .font(.system(size: 12, weight: .semibold))
                .foregroundStyle(labelColor)
            HoldButton(systemImage: "arrow.up") {
                servo.setCartesian(axis: axisIndex, direction: +1)
            } onRelease: {
                servo.setCartesian(axis: axisIndex, direction: 0)
            }
            .disabled(!isRunning)
            HoldButton(systemImage: "arrow.down") {
                servo.setCartesian(axis: axisIndex, direction: -1)
            } onRelease: {
                servo.setCartesian(axis: axisIndex, direction: 0)
            }
            .disabled(!isRunning)
        }
        .frame(width: 60)
    }
}

// MARK: - Hold Button

/// Calls `onPress()` on mouse-down and `onRelease()` on mouse-up.
/// Uses a zero-distance DragGesture so it fires immediately without
/// requiring movement, while still releasing cleanly on mouse-up.
private struct HoldButton: View {
    let systemImage: String
    let onPress:     () -> Void
    let onRelease:   () -> Void

    @State private var isPressed = false

    var body: some View {
        Image(systemName: systemImage)
            .font(.system(size: 13, weight: .semibold))
            .frame(width: 28, height: 26)
            .background(
                isPressed ? Color.accentColor.opacity(0.2) : Color.secondary.opacity(0.1),
                in: RoundedRectangle(cornerRadius: 5)
            )
            .contentShape(RoundedRectangle(cornerRadius: 5))
            .onLongPressGesture(minimumDuration: .infinity, maximumDistance: 50, pressing: { isPressing in
                isPressed = isPressing
                if isPressing { onPress() } else { onRelease() }
            }, perform: {})
    }
}
