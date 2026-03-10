// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "URDFKit",
    products: [
        .library(name: "URDFKit", targets: ["URDFKit"])
    ],
    targets: [
        .target(
            name: "URDFKit",
            path: "Sources/URDFKit",
            swiftSettings: [.swiftLanguageMode(.v5)]
        ),
        .testTarget(
            name: "URDFKitTests",
            dependencies: ["URDFKit"],
            path: "Tests/URDFKitTests",
            swiftSettings: [.swiftLanguageMode(.v5)]
        )
    ]
)
