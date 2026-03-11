import SwiftUI
import URDFKit

// MARK: - Setup Assistant (top-level sheet)

struct SetupAssistantView: View {
    @EnvironmentObject var appState: AppState
    @Binding var isPresented: Bool

    // Wizard state
    @State private var step: Step = .selfCollision
    @State private var setup: RobotSetup = .empty

    // Step 1 — ACM
    @State private var acmBusy    = false
    @State private var acmDone    = false

    // Step 2 — Planning groups
    @State private var editingGroup: PlanningGroup? = nil
    @State private var showGroupEditor = false

    enum Step: Int, CaseIterable {
        case selfCollision = 0
        case planningGroups
        case endEffectors
        case review

        var label: String {
            switch self {
            case .selfCollision:  return "Self-Collision"
            case .planningGroups: return "Planning Groups"
            case .endEffectors:   return "End Effectors"
            case .review:         return "Review"
            }
        }
    }

    var body: some View {
        VStack(spacing: 0) {
            // ── Header / step nav ──────────────────────────────────────────
            HStack(spacing: 0) {
                ForEach(Step.allCases, id: \.rawValue) { s in
                    Button {
                        withAnimation(.easeInOut(duration: 0.2)) { step = s }
                    } label: {
                        VStack(spacing: 4) {
                            Text("\(s.rawValue + 1)")
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .frame(width: 22, height: 22)
                                .background(step == s ? Color.accentColor : Color.secondary.opacity(0.3),
                                            in: Circle())
                                .foregroundStyle(step == s ? .white : .secondary)
                            Text(s.label)
                                .font(.caption)
                                .foregroundStyle(step == s ? .primary : .secondary)
                        }
                    }
                    .buttonStyle(.plain)
                    .frame(maxWidth: .infinity)
                    if s != Step.allCases.last {
                        Divider().frame(maxWidth: 40)
                    }
                }
            }
            .padding(.vertical, 12)
            .padding(.horizontal, 16)
            .background(.bar)

            Divider()

            // ── Page content ───────────────────────────────────────────────
            Group {
                switch step {
                case .selfCollision:  selfCollisionPage
                case .planningGroups: planningGroupsPage
                case .endEffectors:   endEffectorsPage
                case .review:         reviewPage
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)

            Divider()

            // ── Footer nav ─────────────────────────────────────────────────
            HStack {
                Button("Cancel") { isPresented = false }
                Spacer()
                if step.rawValue > 0 {
                    Button("Back") {
                        withAnimation { step = Step(rawValue: step.rawValue - 1)! }
                    }
                }
                if step == .review {
                    Button("Apply") { apply() }
                        .keyboardShortcut(.return, modifiers: [])
                        .buttonStyle(.borderedProminent)
                } else {
                    Button("Next") {
                        withAnimation { step = Step(rawValue: step.rawValue + 1)! }
                    }
                    .buttonStyle(.borderedProminent)
                    .keyboardShortcut(.return, modifiers: [])
                }
            }
            .padding(16)
            .background(.bar)
        }
        .frame(width: 680, height: 540)
        .onAppear {
            setup = appState.robotSetup
        }
        .sheet(isPresented: $showGroupEditor, onDismiss: { editingGroup = nil }) {
            if let model = appState.robotModel {
                PlanningGroupEditor(
                    group:   editingGroup ?? PlanningGroup(name: "", joints: [], tipLink: ""),
                    model:   model,
                    isNew:   editingGroup == nil
                ) { saved in
                    if let idx = setup.planningGroups.firstIndex(where: { $0.id == saved.id }) {
                        setup.planningGroups[idx] = saved
                    } else {
                        setup.planningGroups.append(saved)
                    }
                }
            }
        }
    }

    // MARK: - Step 1: Self-Collision ─────────────────────────────────────────

    private var selfCollisionPage: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Self-Collision Matrix")
                .font(.headline)
            Text("Sample the robot's workspace to find link pairs that can never collide. These are disabled during planning to speed up collision checking.")
                .font(.callout)
                .foregroundStyle(.secondary)

            HStack {
                Button(acmDone ? "Re-Generate" : "Generate") {
                    generateACM()
                }
                .disabled(acmBusy || appState.robotModel == nil)
                if acmBusy {
                    ProgressView().controlSize(.small).padding(.leading, 4)
                }
                if acmDone {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundStyle(.green)
                    Text("\(setup.disabledCollisions.count) pairs disabled")
                        .font(.callout)
                        .foregroundStyle(.secondary)
                }
                Spacer()
            }

            if !setup.disabledCollisions.isEmpty {
                List(setup.disabledCollisions) { pair in
                    HStack {
                        Text(pair.link1).font(.callout.monospaced()).frame(maxWidth: .infinity, alignment: .leading)
                        Text(pair.link2).font(.callout.monospaced()).frame(maxWidth: .infinity, alignment: .leading)
                        Text(pair.reason.rawValue)
                            .font(.callout)
                            .foregroundStyle(reasonColor(pair.reason))
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
                .listStyle(.inset)
                .frame(maxHeight: .infinity)
            } else {
                ContentUnavailableView("No data yet",
                    systemImage: "checkerboard.shield",
                    description: Text("Click Generate to compute the matrix."))
                    .frame(maxHeight: .infinity)
            }
        }
        .padding(16)
    }

    private func reasonColor(_ r: DisabledCollisionPair.DisableReason) -> Color {
        switch r {
        case .adjacent:            return .secondary
        case .defaultInCollision:  return .orange
        case .neverInCollision:    return .blue
        case .userDisabled:        return .red
        }
    }

    private func generateACM() {
        guard let model = appState.robotModel else { return }
        acmBusy = true
        Task.detached(priority: .userInitiated) {
            let pairs = ACMGenerator.compute(model: model)
            await MainActor.run {
                setup.disabledCollisions = pairs
                acmBusy = false
                acmDone = true
            }
        }
    }

    // MARK: - Step 2: Planning Groups ────────────────────────────────────────

    private var planningGroupsPage: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Planning Groups")
                .font(.headline)
            Text("Define kinematic chains used for motion planning. Each group specifies a base link, tip link, and the joints between them.")
                .font(.callout)
                .foregroundStyle(.secondary)

            HStack {
                Button("Add Group") {
                    editingGroup = nil
                    showGroupEditor = true
                }
                Spacer()
            }

            if setup.planningGroups.isEmpty {
                ContentUnavailableView("No Groups",
                    systemImage: "arrow.triangle.branch",
                    description: Text("Add a planning group to enable motion planning."))
                    .frame(maxHeight: .infinity)
            } else {
                List {
                    ForEach(setup.planningGroups) { (group: PlanningGroup) in
                        HStack {
                            VStack(alignment: .leading, spacing: 2) {
                                Text(group.name).font(.headline)
                                Text("\(group.joints.count) joints  •  tip: \(group.tipLink)")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button("Edit") {
                                editingGroup = group
                                showGroupEditor = true
                            }
                            .buttonStyle(.plain)
                            .foregroundStyle(.tint)
                        }
                    }
                    .onDelete { indices in
                        setup.planningGroups.remove(atOffsets: indices)
                    }
                }
                .listStyle(.inset)
                .frame(maxHeight: .infinity)
            }
        }
        .padding(16)
    }

    // MARK: - Step 3: End Effectors ──────────────────────────────────────────

    private var endEffectorsPage: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("End Effectors")
                .font(.headline)
            Text("The IK tip link is the link whose pose is controlled during inverse kinematics. It is set per planning group.")
                .font(.callout)
                .foregroundStyle(.secondary)

            if setup.planningGroups.isEmpty {
                ContentUnavailableView("No Planning Groups",
                    systemImage: "exclamationmark.triangle",
                    description: Text("Define at least one planning group first."))
                    .frame(maxHeight: .infinity)
            } else {
                List {
                    ForEach(setup.planningGroups.indices, id: \.self) { i in
                        if let model = appState.robotModel {
                            let links = chainLinks(group: setup.planningGroups[i], model: model)
                            HStack {
                                Text(setup.planningGroups[i].name)
                                    .frame(width: 140, alignment: .leading)
                                Picker("Tip", selection: $setup.planningGroups[i].tipLink) {
                                    ForEach(links, id: \.self) { Text($0).tag($0) }
                                }
                                .labelsHidden()
                            }
                        }
                    }
                }
                .listStyle(.inset)
                .frame(maxHeight: .infinity)
            }
        }
        .padding(16)
    }

    private func chainLinks(group: PlanningGroup, model: RobotModel) -> [String] {
        // Collect all child links of joints in the group.
        var links = Set<String>()
        for name in group.joints {
            if let j = model.joints[name] {
                links.insert(j.parentLinkName)
                links.insert(j.childLinkName)
            }
        }
        return links.sorted()
    }

    // MARK: - Step 4: Review ─────────────────────────────────────────────────

    private var reviewPage: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                Text("Review Configuration")
                    .font(.headline)

                GroupBox("Planning Groups (\(setup.planningGroups.count))") {
                    if setup.planningGroups.isEmpty {
                        Text("None defined").foregroundStyle(.secondary)
                    } else {
                        VStack(alignment: .leading, spacing: 6) {
                            ForEach(setup.planningGroups) { (g: PlanningGroup) in
                                HStack {
                                    Text(g.name).bold()
                                    Text("— \(g.joints.count) joints, tip: \(g.tipLink)")
                                        .foregroundStyle(.secondary)
                                }
                                .font(.callout)
                            }
                        }
                    }
                }

                GroupBox("Disabled Collision Pairs (\(setup.disabledCollisions.count))") {
                    let byReason = Dictionary(grouping: setup.disabledCollisions, by: \.reason)
                    VStack(alignment: .leading, spacing: 4) {
                        ForEach(DisabledCollisionPair.DisableReason.allCases, id: \.self) { reason in
                            if let pairs = byReason[reason] {
                                HStack {
                                    Circle().fill(reasonColor(reason)).frame(width: 8, height: 8)
                                    Text("\(reason.rawValue): \(pairs.count) pairs")
                                        .font(.callout)
                                }
                            }
                        }
                    }
                }

                GroupBox("Active Planning Group") {
                    Picker("Active Group", selection: $setup.activePlanningGroup) {
                        Text("None").tag(Optional<String>.none)
                        ForEach(setup.planningGroups) { (g: PlanningGroup) in
                            Text(g.name).tag(Optional(g.name))
                        }
                    }
                    .labelsHidden()
                    Text("The active group's joints are used for planning and IK.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
            .padding(16)
        }
    }

    // MARK: - Apply ────────────────────────────────────────────────────────

    private func apply() {
        appState.robotSetup = setup
        isPresented = false
    }
}

// MARK: - Planning Group Editor (sheet)

private struct PlanningGroupEditor: View {
    @Environment(\.dismiss) private var dismiss

    @State private var group: PlanningGroup
    let model: RobotModel
    let isNew: Bool
    let onSave: (PlanningGroup) -> Void

    @State private var baseLink: String = ""
    @State private var tipLink:  String = ""

    private var allLinks: [String] { model.orderedJointNames
        .compactMap { model.joints[$0]?.childLinkName }
        .reduce([model.rootLinkName]) { $0 + [$1] }
    }

    init(group: PlanningGroup, model: RobotModel, isNew: Bool, onSave: @escaping (PlanningGroup) -> Void) {
        _group = State(initialValue: group)
        self.model  = model
        self.isNew  = isNew
        self.onSave = onSave
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            Text(isNew ? "Add Planning Group" : "Edit Planning Group")
                .font(.headline)

            TextField("Group Name", text: $group.name)
                .textFieldStyle(.roundedBorder)

            HStack(spacing: 16) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Base Link").font(.caption).foregroundStyle(.secondary)
                    Picker("Base", selection: $baseLink) {
                        ForEach(allLinks, id: \.self) { Text($0).tag($0) }
                    }
                    .labelsHidden()
                    .frame(minWidth: 160)
                }
                Image(systemName: "arrow.right")
                    .foregroundStyle(.secondary)
                VStack(alignment: .leading, spacing: 4) {
                    Text("Tip Link").font(.caption).foregroundStyle(.secondary)
                    Picker("Tip", selection: $tipLink) {
                        ForEach(allLinks, id: \.self) { Text($0).tag($0) }
                    }
                    .labelsHidden()
                    .frame(minWidth: 160)
                }
            }

            let derivedJoints = !baseLink.isEmpty && !tipLink.isEmpty
                ? PlanningGroup.joints(from: baseLink, to: tipLink, model: model)
                : []

            GroupBox("Joints in Chain (\(derivedJoints.count))") {
                if derivedJoints.isEmpty {
                    Text("Select base and tip links above.")
                        .foregroundStyle(.secondary)
                        .font(.callout)
                } else {
                    FlowLayout(spacing: 6) {
                        ForEach(derivedJoints, id: \.self) {
                            Text($0)
                                .font(.system(size: 11, design: .monospaced))
                                .padding(.horizontal, 6).padding(.vertical, 2)
                                .background(.quaternary, in: RoundedRectangle(cornerRadius: 4))
                        }
                    }
                }
            }
            .frame(minHeight: 60)

            Spacer()

            HStack {
                Button("Cancel") { dismiss() }
                Spacer()
                Button("Save") {
                    group.joints  = derivedJoints
                    group.tipLink = tipLink.isEmpty ? (derivedJoints.isEmpty ? "" : model.joints[derivedJoints.last!]?.childLinkName ?? "") : tipLink
                    onSave(group)
                    dismiss()
                }
                .buttonStyle(.borderedProminent)
                .disabled(group.name.isEmpty || derivedJoints.isEmpty)
            }
        }
        .padding(20)
        .frame(width: 420, height: 380)
        .onAppear {
            baseLink = baseLink.isEmpty ? model.rootLinkName : baseLink
            tipLink  = tipLink.isEmpty  ? (allLinks.last ?? "") : tipLink
        }
    }
}

// MARK: - Simple flow layout (chip wrapping)

private struct FlowLayout: Layout {
    var spacing: CGFloat = 8

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        let width = proposal.width ?? .infinity
        var x: CGFloat = 0; var y: CGFloat = 0; var rowH: CGFloat = 0
        for view in subviews {
            let size = view.sizeThatFits(.unspecified)
            if x + size.width > width && x > 0 { y += rowH + spacing; x = 0; rowH = 0 }
            rowH = max(rowH, size.height); x += size.width + spacing
        }
        return CGSize(width: width, height: y + rowH)
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        var x = bounds.minX; var y = bounds.minY; var rowH: CGFloat = 0
        for view in subviews {
            let size = view.sizeThatFits(.unspecified)
            if x + size.width > bounds.maxX && x > bounds.minX {
                y += rowH + spacing; x = bounds.minX; rowH = 0
            }
            view.place(at: CGPoint(x: x, y: y), proposal: ProposedViewSize(size))
            rowH = max(rowH, size.height); x += size.width + spacing
        }
    }
}

