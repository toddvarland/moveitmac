import SwiftUI
import AppKit

// MARK: - Panel

struct JointSliderPanel: View {
    @EnvironmentObject var appState: AppState

    var body: some View {
        Group {
            if appState.isRobotLoaded {
                // ScrollView + VStack avoids NSTableView (List) entirely.
                // NSTableView can interfere with NSSlider mouse-tracking inside cells.
                ScrollView(.vertical) {
                    VStack(spacing: 0) {
                        ForEach(appState.orderedJointNames, id: \.self) { name in
                            JointRowControl(
                                name:     name,
                                range:    appState.jointLimits[name] ?? (-.pi ... .pi),
                                appState: appState
                            )
                            Divider()
                        }
                    }
                    .padding(.vertical, 4)
                }
            } else {
                ContentUnavailableView(
                    "No Robot",
                    systemImage: "slider.horizontal.3",
                    description: Text("Joint sliders appear here after loading a URDF.")
                )
            }
        }
        .frame(minWidth: 220)
        .navigationTitle("Joints")
    }
}

// MARK: - Per-joint NSViewRepresentable

/// A fully-AppKit row: name label + NSSlider + value label in a single NSView.
///
/// Key design choices:
/// - No @Binding, no @State — zero SwiftUI state mutations during drag.
/// - Coordinator writes directly to appState.jointAngles (plain var, no objectWillChange).
/// - valueLabel updated via direct NSTextField.stringValue — no SwiftUI re-render path.
/// - updateNSView only touches range bounds, never slider.doubleValue.
///   This means even at 10 Hz re-renders (from collision publishes) nothing can fight
///   AppKit's mouse-tracking runloop.
private struct JointRowControl: NSViewRepresentable {
    let name:     String
    let range:    ClosedRange<Double>
    let appState: AppState      // plain reference — not observed, doesn't trigger re-renders

    func makeCoordinator() -> Coordinator {
        Coordinator(name: name, appState: appState)
    }

    func makeNSView(context: Context) -> JointRowView {
        let initial = appState.jointAngles[name] ?? 0
        let row = JointRowView(name: name, range: range, initialValue: initial)
        row.slider.target = context.coordinator
        row.slider.action = #selector(Coordinator.sliderMoved(_:))
        context.coordinator.valueLabel = row.valueLabel
        return row
    }

    func updateNSView(_ nsView: JointRowView, context: Context) {
        // Called at most ~10 Hz from collision publishes.
        // Only safe, non-drag-interfering updates here.
        nsView.slider.minValue = range.lowerBound
        nsView.slider.maxValue = range.upperBound
        context.coordinator.name = name
    }

    // MARK: Coordinator

    final class Coordinator: NSObject {
        var name:     String
        let appState: AppState
        weak var valueLabel: NSTextField?

        init(name: String, appState: AppState) {
            self.name     = name
            self.appState = appState
        }

        @objc func sliderMoved(_ sender: NSSlider) {
            let v = sender.doubleValue
            appState.jointAngles[name] = v
            valueLabel?.stringValue = Self.format(v)
        }

        static func format(_ radians: Double) -> String {
            String(format: "%.1f°", radians * 180 / .pi)
        }
    }
}

// MARK: - AppKit row view

/// Pure NSView: name label (top-left) + degree label (top-right) + slider (bottom).
/// Created once per joint; never torn down during normal operation.
final class JointRowView: NSView {
    let nameLabel  = NSTextField(labelWithString: "")
    let valueLabel = NSTextField(labelWithString: "")
    let slider     = NSSlider()

    init(name: String, range: ClosedRange<Double>, initialValue: Double) {
        super.init(frame: .zero)

        nameLabel.stringValue    = name
        nameLabel.font           = .systemFont(ofSize: NSFont.smallSystemFontSize)
        nameLabel.textColor      = .secondaryLabelColor
        nameLabel.lineBreakMode  = .byTruncatingMiddle
        nameLabel.setContentCompressionResistancePriority(.defaultLow, for: .horizontal)

        valueLabel.stringValue   = JointRowControl.Coordinator.format(initialValue)
        valueLabel.font          = .monospacedDigitSystemFont(
                                       ofSize: NSFont.smallSystemFontSize, weight: .regular)
        valueLabel.alignment     = .right
        valueLabel.setContentCompressionResistancePriority(.required, for: .horizontal)

        slider.minValue          = range.lowerBound
        slider.maxValue          = range.upperBound
        slider.doubleValue       = initialValue
        slider.isContinuous      = true

        [nameLabel, valueLabel, slider].forEach {
            $0.translatesAutoresizingMaskIntoConstraints = false
            addSubview($0)
        }

        NSLayoutConstraint.activate([
            nameLabel.leadingAnchor.constraint(equalTo: leadingAnchor, constant: 8),
            nameLabel.topAnchor.constraint(equalTo: topAnchor, constant: 6),
            nameLabel.trailingAnchor.constraint(
                lessThanOrEqualTo: valueLabel.leadingAnchor, constant: -4),

            valueLabel.trailingAnchor.constraint(equalTo: trailingAnchor, constant: -8),
            valueLabel.centerYAnchor.constraint(equalTo: nameLabel.centerYAnchor),
            valueLabel.widthAnchor.constraint(greaterThanOrEqualToConstant: 50),

            slider.leadingAnchor.constraint(equalTo: leadingAnchor, constant: 8),
            slider.trailingAnchor.constraint(equalTo: trailingAnchor, constant: -8),
            slider.topAnchor.constraint(equalTo: nameLabel.bottomAnchor, constant: 2),
            slider.bottomAnchor.constraint(equalTo: bottomAnchor, constant: -6),
        ])
    }

    required init?(coder: NSCoder) { fatalError() }
}
