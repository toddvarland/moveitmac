import Testing
import Foundation
@testable import MoveItMac

@Suite("AppState")
struct AppStateTests {

    @Test("Initial state has no robot loaded")
    @MainActor
    func initialStateEmpty() {
        let state = AppState()
        #expect(state.isRobotLoaded == false)
        #expect(state.robotName == nil)
        #expect(state.jointAngles.isEmpty)
        #expect(state.obstacles.isEmpty)
    }

    @Test("Ordered joint names is empty before load")
    @MainActor
    func orderedJointsBeforeLoad() {
        let state = AppState()
        #expect(state.orderedJointNames.isEmpty)
    }
}
