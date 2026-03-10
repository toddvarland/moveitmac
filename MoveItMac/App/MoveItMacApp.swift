import SwiftUI

@main
struct MoveItMacApp: App {
    @StateObject private var appState = AppState()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(appState)
        }
        .windowStyle(.titleBar)
        .windowToolbarStyle(.unified)
        .commands {
            CommandGroup(replacing: .newItem) { }
            CommandMenu("Robot") {
                Button("Load URDF…") {
                    appState.openURDFPanel()
                }
                .keyboardShortcut("o")
            }
        }
    }
}
