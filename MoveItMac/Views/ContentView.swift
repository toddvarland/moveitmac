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
                .overlay(alignment: .bottomLeading) {
                    AxisLegend()
                        .padding(10)
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

private struct AxisLegend: View {
    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            legendRow(color: .red,   label: "X")
            legendRow(color: .green, label: "Y")
            legendRow(color: .blue,  label: "Z")
        }
        .padding(8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }

    private func legendRow(color: Color, label: String) -> some View {
        HStack(spacing: 6) {
            RoundedRectangle(cornerRadius: 2)
                .fill(color)
                .frame(width: 18, height: 4)
            Text(label)
                .font(.system(size: 11, weight: .bold, design: .monospaced))
                .foregroundStyle(color)
        }
    }
}
