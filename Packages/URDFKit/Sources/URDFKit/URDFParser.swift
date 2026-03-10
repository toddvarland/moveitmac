import Foundation
import simd

// MARK: - Errors

public enum URDFError: LocalizedError {
    case fileNotFound(URL)
    case xmlParseError(String)
    case missingRobotElement
    case missingAttribute(element: String, attribute: String)
    case unknownJointType(String)

    public var errorDescription: String? {
        switch self {
        case .fileNotFound(let url):
            return "URDF file not found: \(url.path)"
        case .xmlParseError(let msg):
            return "XML parse error: \(msg)"
        case .missingRobotElement:
            return "URDF is missing the <robot> root element"
        case .missingAttribute(let el, let attr):
            return "Missing required attribute '\(attr)' on <\(el)>"
        case .unknownJointType(let t):
            return "Unknown joint type: '\(t)'"
        }
    }
}

// MARK: - Public API

public final class URDFParser: NSObject {

    /// Parse a URDF file at the given URL and return a `RobotModel`.
    public static func parse(contentsOf url: URL) throws -> RobotModel {
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw URDFError.fileNotFound(url)
        }
        let impl = URDFParserImpl()
        return try impl.parse(url: url)
    }
}

// MARK: - Implementation (internal, XMLParserDelegate)

private final class URDFParserImpl: NSObject, XMLParserDelegate {

    // Accumulated model data
    private var robotName = ""
    private var links: [String: Link] = [:]
    private var joints: [String: Joint] = [:]

    // Parsing stack and current builders
    private var elementStack: [String] = []
    private var currentLink: LinkBuilder?
    private var currentJoint: JointBuilder?
    private var currentVisual: VisualBuilder?
    private var currentCollision: CollisionBuilder?
    private var currentInertial: InertialBuilder?
    private var parseError: Error?

    func parse(url: URL) throws -> RobotModel {
        guard let xmlParser = XMLParser(contentsOf: url) else {
            throw URDFError.xmlParseError("Could not initialise XML parser for \(url.lastPathComponent)")
        }
        xmlParser.delegate = self
        xmlParser.parse()

        if let err = parseError { throw err }
        guard !robotName.isEmpty else { throw URDFError.missingRobotElement }

        // Root link = link with no parent joint
        let childLinks = Set(joints.values.map { $0.childLinkName })
        let rootName = links.keys
            .filter { !childLinks.contains($0) }
            .sorted()
            .first ?? ""

        return RobotModel(
            name: robotName,
            links: links,
            joints: joints,
            rootLinkName: rootName
        )
    }

    // MARK: - XMLParserDelegate

    func parser(
        _ parser: XMLParser,
        didStartElement elementName: String,
        namespaceURI: String?,
        qualifiedName _: String?,
        attributes: [String: String]
    ) {
        elementStack.append(elementName)

        switch elementName {

        case "robot":
            robotName = attributes["name"] ?? "unnamed_robot"

        case "link":
            guard let name = attributes["name"] else { return }
            currentLink = LinkBuilder(name: name)

        case "joint":
            guard
                let name = attributes["name"],
                let typeStr = attributes["type"]
            else { return }
            guard let type = JointType(rawValue: typeStr) else {
                parseError = URDFError.unknownJointType(typeStr)
                parser.abortParsing()
                return
            }
            currentJoint = JointBuilder(name: name, type: type)

        case "visual":
            currentVisual = VisualBuilder()

        case "collision":
            currentCollision = CollisionBuilder()

        case "inertial":
            currentInertial = InertialBuilder()

        case "origin":
            let t = Transform(
                xyz: parseVec3(attributes["xyz"]),
                rpy: parseVec3(attributes["rpy"])
            )
            switch parentElement() {
            case "visual":    currentVisual?.origin = t
            case "collision": currentCollision?.origin = t
            case "inertial":  currentInertial?.origin = t
            case "joint":     currentJoint?.origin = t
            default: break
            }

        case "axis":
            currentJoint?.axis = parseVec3(attributes["xyz"])

        case "parent":
            currentJoint?.parentLinkName = attributes["link"] ?? ""

        case "child":
            currentJoint?.childLinkName = attributes["link"] ?? ""

        case "limit":
            currentJoint?.limits = JointLimits(
                lower:    Double(attributes["lower"]    ?? "0") ?? 0,
                upper:    Double(attributes["upper"]    ?? "0") ?? 0,
                effort:   Double(attributes["effort"]   ?? "0") ?? 0,
                velocity: Double(attributes["velocity"] ?? "0") ?? 0
            )

        case "mass":
            currentInertial?.mass = Double(attributes["value"] ?? "0") ?? 0

        case "inertia":
            currentInertial?.ixx = Double(attributes["ixx"] ?? "0") ?? 0
            currentInertial?.ixy = Double(attributes["ixy"] ?? "0") ?? 0
            currentInertial?.ixz = Double(attributes["ixz"] ?? "0") ?? 0
            currentInertial?.iyy = Double(attributes["iyy"] ?? "0") ?? 0
            currentInertial?.iyz = Double(attributes["iyz"] ?? "0") ?? 0
            currentInertial?.izz = Double(attributes["izz"] ?? "0") ?? 0

        case "material":
            // Visual material name (optional)
            if let name = attributes["name"] {
                currentVisual?.materialName = name
            }

        case "mesh":
            let filename = attributes["filename"] ?? ""
            let scale = attributes["scale"].map { parseVec3($0) } ?? SIMD3<Double>(1, 1, 1)
            applyGeometry(.mesh(filename: filename, scale: scale))

        case "box":
            if let sizeStr = attributes["size"] {
                applyGeometry(.box(size: parseVec3(sizeStr)))
            }

        case "cylinder":
            applyGeometry(.cylinder(
                radius: Double(attributes["radius"] ?? "0") ?? 0,
                length: Double(attributes["length"] ?? "0") ?? 0
            ))

        case "sphere":
            applyGeometry(.sphere(radius: Double(attributes["radius"] ?? "0") ?? 0))

        default:
            break
        }
    }

    func parser(
        _ parser: XMLParser,
        didEndElement elementName: String,
        namespaceURI: String?,
        qualifiedName _: String?
    ) {
        defer { elementStack.removeLast() }

        switch elementName {
        case "visual":
            currentLink?.visual = currentVisual?.build()
            currentVisual = nil

        case "collision":
            currentLink?.collision = currentCollision?.build()
            currentCollision = nil

        case "inertial":
            currentLink?.inertial = currentInertial?.build()
            currentInertial = nil

        case "link":
            if let link = currentLink?.build() {
                links[link.name] = link
            }
            currentLink = nil

        case "joint":
            if let joint = currentJoint?.build() {
                joints[joint.name] = joint
            }
            currentJoint = nil

        default:
            break
        }
    }

    func parser(_ parser: XMLParser, parseErrorOccurred error: Error) {
        parseError = URDFError.xmlParseError(error.localizedDescription)
    }

    // MARK: - Helpers

    /// The element one level above the current top of the stack.
    private func parentElement() -> String? {
        guard elementStack.count >= 2 else { return nil }
        return elementStack[elementStack.count - 2]
    }

    /// Find the nearest ancestor that is "visual" or "collision".
    private func geometryContainer() -> String? {
        elementStack.reversed().first(where: { $0 == "visual" || $0 == "collision" })
    }

    private func applyGeometry(_ geo: Geometry) {
        switch geometryContainer() {
        case "visual":    currentVisual?.geometry = geo
        case "collision": currentCollision?.geometry = geo
        default: break
        }
    }

    private func parseVec3(_ str: String?) -> SIMD3<Double> {
        guard let str else { return .zero }
        let parts = str.split(separator: " ").compactMap { Double($0) }
        guard parts.count >= 3 else { return .zero }
        return SIMD3(parts[0], parts[1], parts[2])
    }
}

// MARK: - Mutable Builders (used during SAX parsing)

private final class LinkBuilder {
    let name: String
    var visual: Visual?
    var collision: CollisionGeometry?
    var inertial: Inertial?
    init(name: String) { self.name = name }
    func build() -> Link {
        Link(name: name, visual: visual, collision: collision, inertial: inertial)
    }
}

private final class JointBuilder {
    let name: String
    let type: JointType
    var parentLinkName = ""
    var childLinkName = ""
    var origin = Transform.identity
    var axis = SIMD3<Double>(0, 0, 1)
    var limits: JointLimits?
    init(name: String, type: JointType) { self.name = name; self.type = type }
    func build() -> Joint? {
        guard !parentLinkName.isEmpty, !childLinkName.isEmpty else { return nil }
        return Joint(
            name: name, type: type,
            parentLinkName: parentLinkName, childLinkName: childLinkName,
            origin: origin, axis: axis, limits: limits
        )
    }
}

private final class VisualBuilder {
    var origin = Transform.identity
    var geometry: Geometry?
    var materialName: String?
    func build() -> Visual? {
        guard let geo = geometry else { return nil }
        return Visual(origin: origin, geometry: geo, materialName: materialName)
    }
}

private final class CollisionBuilder {
    var origin = Transform.identity
    var geometry: Geometry?
    func build() -> CollisionGeometry? {
        guard let geo = geometry else { return nil }
        return CollisionGeometry(origin: origin, geometry: geo)
    }
}

private final class InertialBuilder {
    var origin = Transform.identity
    var mass = 0.0
    var ixx = 0.0, ixy = 0.0, ixz = 0.0
    var iyy = 0.0, iyz = 0.0, izz = 0.0
    func build() -> Inertial {
        Inertial(
            origin: origin, mass: mass,
            ixx: ixx, ixy: ixy, ixz: ixz,
            iyy: iyy, iyz: iyz, izz: izz
        )
    }
}
