import SwiftUI

struct ContentView: View {
    @EnvironmentObject var appState: AppState

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
