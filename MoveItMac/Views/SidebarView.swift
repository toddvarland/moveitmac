import SwiftUI

struct SidebarView: View {
    @EnvironmentObject var appState: AppState
    @State private var addObstacleSheet = false

    var body: some View {
        List {
            // ── Robot ──────────────────────────────────────────────────────
            Section("Robot") {
                if let name = appState.robotName {
                    Label(name, systemImage: "gearshape.2.fill")
                        .foregroundStyle(.primary)
                    // Collision status — updated at ~120 Hz, only redraws when state changes.
                    if appState.collisionResult.hasCollision {
                        Label("Collision detected", systemImage: "exclamationmark.triangle.fill")
                            .foregroundStyle(.red)
                            .font(.caption.bold())
                    } else {
                        Label("Clear", systemImage: "checkmark.circle.fill")
                            .foregroundStyle(.green)
                            .font(.caption)
                    }
                } else {
                    Label("None loaded", systemImage: "gearshape.2")
                        .foregroundStyle(.secondary)
                }
            }

            // ── Planner ────────────────────────────────────────────────────
            if appState.isRobotLoaded {
                Section("Planner") {
                    HStack(spacing: 6) {
                        Button("Set Start") { appState.setPlanStart() }
                            .buttonStyle(.bordered)
                        Button("Set Goal")  { appState.setPlanGoal() }
                            .buttonStyle(.bordered)
                    }
                    .font(.caption)

                    if !appState.planStart.isEmpty {
                        Label("Start set", systemImage: "circle.fill")
                            .font(.caption).foregroundStyle(.green)
                    }
                    if !appState.planGoal.isEmpty {
                        Label("Goal set", systemImage: "target")
                            .font(.caption).foregroundStyle(.blue)
                    }

                    Button {
                        appState.startPlanning()
                    } label: {
                        if appState.plannerStatus == .planning {
                            HStack(spacing: 6) {
                                ProgressView().scaleEffect(0.65)
                                Text("Planning…")
                            }
                        } else {
                            Text("Plan")
                        }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(!appState.canPlan)

                    if !appState.plannerMessage.isEmpty {
                        Text(appState.plannerMessage)
                            .font(.caption)
                            .foregroundStyle(
                                appState.plannerStatus == .done    ? Color.green  :
                                appState.plannerStatus == .failed  ? Color.red    :
                                Color.secondary
                            )
                    }

                    if appState.plannerStatus == .done {
                        Button("Apply Goal") {
                            appState.jointAngles = appState.planGoal
                        }
                        .font(.caption)
                    }
                }
            }

            // ── Playback ───────────────────────────────────────────────────
            if appState.canPlay {
                Section("Trajectory") {
                    HStack {
                        Button {
                            appState.playPause()
                        } label: {
                            Image(systemName: appState.isPlaying ? "pause.fill" : "play.fill")
                                .frame(width: 24)
                        }
                        .buttonStyle(.bordered)
                        .keyboardShortcut(.space, modifiers: [])

                        Slider(value: Binding(
                            get: { appState.playbackProgress },
                            set: { appState.scrubTo($0) }
                        ))
                    }

                    HStack {
                        Text("Speed")
                            .font(.caption)
                        Slider(value: $appState.playbackSpeed, in: 0.1...5,
                               label: { EmptyView() })
                        Text(String(format: "%.1f×", appState.playbackSpeed))
                            .font(.caption.monospacedDigit())
                            .frame(width: 38, alignment: .trailing)
                    }

                    let idx   = appState.playbackIndex
                    let total = max(appState.trajectory.count - 1, 1)
                    if let dur = appState.timedTrajectory?.duration {
                        let remaining = dur / max(appState.playbackSpeed, 0.01) * (1.0 - appState.playbackProgress)
                        Text(String(format: "%.1f s total  •  %.1f s left", dur, remaining))
                            .font(.caption.monospacedDigit())
                            .foregroundStyle(.secondary)
                    } else {
                        Text("Waypoint \(idx + 1) / \(total + 1)")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
            }

            // ── IK ─────────────────────────────────────────────────────────
            if appState.isRobotLoaded {
                Section("Inverse Kinematics") {
                    Toggle("IK Mode", isOn: $appState.useIK)
                    if appState.useIK {
                        let links = appState.robotModel?.links.keys.sorted() ?? []
                        Picker("End Effector", selection: $appState.ikEndEffectorLink) {
                            ForEach(links, id: \.self) { Text($0).tag($0) }
                        }
                        .pickerStyle(.menu)
                        .font(.caption)
                    }
                }
            }

            // ── Obstacles ──────────────────────────────────────────────────
            Section {
                if appState.obstacles.isEmpty {
                    Text("No obstacles")
                        .foregroundStyle(.tertiary)
                        .font(.caption)
                } else {
                    ForEach(appState.obstacles) { obstacle in
                        ObstacleRow(obstacle: obstacle) {
                            appState.obstacles.removeAll { $0.id == obstacle.id }
                        }
                    }
                    .onDelete { indices in
                        appState.obstacles.remove(atOffsets: indices)
                    }
                }
            } header: {
                HStack {
                    Text("Obstacles")
                    Spacer()
                    Button(action: { addObstacleSheet = true }) {
                        Image(systemName: "plus")
                    }
                    .buttonStyle(.plain)
                    .foregroundStyle(Color.accentColor)
                }
            }
            Section("Axes") {
                AxisLegendRow(color: .red,   label: "X")
                AxisLegendRow(color: .green, label: "Y")
                AxisLegendRow(color: .blue,  label: "Z")
            }
        }
        .listStyle(.sidebar)
        .frame(minWidth: 180)
        .navigationTitle("Scene")
        .sheet(isPresented: $addObstacleSheet) {
            AddObstacleSheet(isPresented: $addObstacleSheet)
                .environmentObject(appState)
        }
    }
}

// MARK: - Obstacle Row

private struct ObstacleRow: View {
    let obstacle: Obstacle
    let onDelete: () -> Void

    var body: some View {
        HStack {
            Label(obstacle.name, systemImage: obstacle.shape.systemImage)
                .font(.caption)
            Spacer()
            Button(action: onDelete) {
                Image(systemName: "minus.circle.fill")
                    .foregroundStyle(.red)
            }
            .buttonStyle(.plain)
        }
    }
}

// MARK: - Add Obstacle Sheet

private struct AddObstacleSheet: View {
    @EnvironmentObject var appState: AppState
    @Binding var isPresented: Bool

    @State private var shape: ObstacleShape = .box
    @State private var name = ""
    @State private var x = 0.5
    @State private var y = 0.0
    @State private var z = 0.5
    @State private var sizeX = 0.1
    @State private var sizeY = 0.1
    @State private var sizeZ = 0.1
    @State private var radius = 0.05
    @State private var height = 0.1

    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            Text("Add Obstacle").font(.headline)

            Picker("Shape", selection: $shape) {
                ForEach(ObstacleShape.allCases) { s in
                    Label(s.label, systemImage: s.systemImage).tag(s)
                }
            }
            .pickerStyle(.segmented)

            TextField("Name", text: $name)
                .textFieldStyle(.roundedBorder)

            GroupBox("Position (m)") {
                VStack(spacing: 6) {
                    LabeledSlider(label: "X", value: $x, range: -2...2, color: .red)
                    LabeledSlider(label: "Y", value: $y, range: -2...2, color: .blue)
                    LabeledSlider(label: "Z", value: $z, range:  0...2, color: .green)
                }
            }

            GroupBox("Dimensions") {
                switch shape {
                case .box:
                    VStack(spacing: 6) {
                        LabeledSlider(label: "W", value: $sizeX, range: 0.01...1)
                        LabeledSlider(label: "D", value: $sizeY, range: 0.01...1)
                        LabeledSlider(label: "H", value: $sizeZ, range: 0.01...1)
                    }
                case .cylinder:
                    VStack(spacing: 6) {
                        LabeledSlider(label: "R",  value: $radius, range: 0.01...0.5)
                        LabeledSlider(label: "H",  value: $height, range: 0.01...2)
                    }
                case .sphere:
                    LabeledSlider(label: "R",  value: $radius, range: 0.01...0.5)
                }
            }

            HStack {
                Spacer()
                Button("Cancel") { isPresented = false }
                    .keyboardShortcut(.escape, modifiers: [])
                Button("Add") { addAndClose() }
                    .keyboardShortcut(.return, modifiers: [])
                    .buttonStyle(.borderedProminent)
            }
        }
        .padding(20)
        .frame(width: 340)
    }

    private func addAndClose() {
        let displayName = name.isEmpty ? shape.label : name
        let obstacle = Obstacle(
            name: displayName,
            shape: shape,
            position: SIMD3<Double>(x, y, z),
            boxSize: SIMD3<Double>(sizeX, sizeY, sizeZ),
            radius: radius,
            height: height
        )
        appState.obstacles.append(obstacle)
        isPresented = false
    }
}

private struct AxisLegendRow: View {
    let color: Color
    let label: String
    var body: some View {
        HStack(spacing: 8) {
            RoundedRectangle(cornerRadius: 2)
                .fill(color)
                .frame(width: 18, height: 4)
            Text(label)
                .font(.system(size: 11, weight: .bold, design: .monospaced))
                .foregroundStyle(color)
        }
    }
}

private struct LabeledSlider: View {
    let label: String
    @Binding var value: Double
    let range: ClosedRange<Double>
    var color: Color? = nil

    var body: some View {
        HStack {
            Text(label)
                .frame(width: 16)
                .font(.caption.bold())
                .foregroundStyle(color ?? .secondary)
            Slider(value: $value, in: range)
            Text(String(format: "%.2f", value))
                .font(.caption.monospacedDigit())
                .frame(width: 38, alignment: .trailing)
        }
    }
}
