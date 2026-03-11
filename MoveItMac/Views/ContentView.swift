import SwiftUI

struct ContentView: View {
    @EnvironmentObject var appState: AppState
    @StateObject private var servo = ServoEngine()
    @State private var showSetupAssistant = false
    @State private var showServoPanel     = false

    var body: some View {
        NavigationSplitView(columnVisibility: .constant(.all)) {
            SidebarView()
                .navigationSplitViewColumnWidth(min: 180, ideal: 200, max: 280)
        } content: {
            RobotSceneView()
                .navigationSplitViewColumnWidth(min: 400, ideal: 700)
                .overlay(alignment: .topLeading) {
                    if !appState.isRobotLoaded {
                        EmptySceneOverlay()
                    }
                }

        } detail: {
            JointSliderPanel()
                .navigationSplitViewColumnWidth(min: 220, ideal: 240, max: 320)
        }
        .toolbar {
            ToolbarItem(placement: .primaryAction) {
                Button("Load URDF…", systemImage: "plus.circle") {
                    appState.openURDFPanel()
                }
            }
            ToolbarItem(placement: .primaryAction) {
                Button("Setup Assistant", systemImage: "slider.horizontal.3") {
                    showSetupAssistant = true
                }
                .disabled(!appState.isRobotLoaded)
            }
            ToolbarItem(placement: .primaryAction) {
                Button("Servo", systemImage: "waveform.path") {
                    showServoPanel.toggle()
                }
                .tint(servo.status == .running ? .green : nil)
                .disabled(!appState.isRobotLoaded)
                .popover(isPresented: $showServoPanel, arrowEdge: .top) {
                    ServoControlPanel(servo: servo)
                        .environmentObject(appState)
                }
            }
        }
        .sheet(isPresented: $showSetupAssistant) {
            SetupAssistantView(isPresented: $showSetupAssistant)
                .environmentObject(appState)
        }
    }
}

private struct EmptySceneOverlay: View {
    var body: some View {
        ContentUnavailableView(
            "No Robot Loaded",
            systemImage: "gearshape.2",
            description: Text("Use Robot → Load URDF… or press ⌘O to open a URDF file.")
        )
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(.ultraThinMaterial)
    }
}

