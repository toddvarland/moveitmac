#!/usr/bin/env bash
# build_deps.sh — Build OMPL, FCL, and KDL as macOS arm64 xcframeworks.
#
# Run once before opening MoveItMac.xcodeproj in Xcode:
#   bash build_deps.sh
#
# Outputs go into ./deps/ (Git-ignored).
# Intermediate build artefacts go into ./.deps_build/ (also Git-ignored).
#
# Re-running is safe: already-built xcframeworks are skipped.
# To force a full rebuild: rm -rf deps .deps_build && bash build_deps.sh
#
# Licenses:
#   OMPL  — BSD-3-Clause  (static or dynamic OK)
#   FCL   — BSD-3-Clause  (static or dynamic OK)
#   KDL   — LGPL-2.1      *** dynamic linking only ***

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$SCRIPT_DIR/deps"
BUILD_DIR="$SCRIPT_DIR/.deps_build"
JOBS="$(sysctl -n hw.logicalcpu)"
DEPLOYMENT_TARGET="26.0"
ARCH="arm64"

# Pinned versions
EIGEN_TAG="3.4.0"
CCD_TAG="v2.1"
FCL_TAG="v0.7.0"
OMPL_TAG="1.6.0"
KDL_TAG="v1.5.1"

# ── Colours ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
log()  { echo -e "${GREEN}[deps]${NC} $*"; }
warn() { echo -e "${YELLOW}[deps]${NC} $*"; }
die()  { echo -e "${RED}[deps] ERROR:${NC} $*" >&2; exit 1; }

# ── Prereq checks ────────────────────────────────────────────────────────────
command -v cmake      >/dev/null || die "cmake not found — run: brew install cmake"
command -v git        >/dev/null || die "git not found"
command -v xcodebuild >/dev/null || die "xcodebuild not found — install Xcode"
command -v xcodegen   >/dev/null || die "xcodegen not found — run: brew install xcodegen"

log "Building C++ dependencies for MoveIt Mac"
log "  Target: macOS ${DEPLOYMENT_TARGET} (${ARCH})"
log "  Jobs:   ${JOBS}"
echo ""

mkdir -p "$DEPS_DIR" "$BUILD_DIR"

# ── Helper: shallow-clone a repo if not already present ──────────────────────
clone_if_needed() {
    local dir="$1" url="$2" tag="$3"
    if [[ ! -d "$dir" ]]; then
        log "Cloning $(basename "$dir") @ ${tag} …"
        git clone --depth 1 --branch "$tag" "$url" "$dir"
    fi
}

# ── Helper: run cmake configure + build + install ────────────────────────────
cmake_build() {
    local src="$1" bld="$2" prefix="$3"
    shift 3
    cmake -S "$src" -B "$bld" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES="$ARCH" \
        -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
        -DCMAKE_INSTALL_PREFIX="$prefix" \
        "$@"
    cmake --build "$bld" --config Release -j"$JOBS"
    cmake --install "$bld"
}

# ────────────────────────────────────────────────────────────────────────────
# 1. Eigen (header-only — no xcframework needed, just include path)
# ────────────────────────────────────────────────────────────────────────────
EIGEN_SRC="$BUILD_DIR/eigen"
clone_if_needed "$EIGEN_SRC" \
    "https://gitlab.com/libeigen/eigen.git" "$EIGEN_TAG"
EIGEN_INCLUDE="$EIGEN_SRC"
log "✓ Eigen headers at $EIGEN_INCLUDE"

# ────────────────────────────────────────────────────────────────────────────
# 2. libccd (required by FCL for convex-hull distance queries)
# ────────────────────────────────────────────────────────────────────────────
CCD_SRC="$BUILD_DIR/libccd"
CCD_BLD="$BUILD_DIR/libccd-build"
CCD_PREFIX="$BUILD_DIR/libccd-install"

clone_if_needed "$CCD_SRC" \
    "https://github.com/danfis/libccd.git" "$CCD_TAG"

if [[ ! -f "$CCD_PREFIX/lib/libccd.dylib" ]]; then
    log "Building libccd …"
    cmake_build "$CCD_SRC" "$CCD_BLD" "$CCD_PREFIX" \
        -DBUILD_SHARED_LIBS=ON \
        -DCCD_HIDE_ALL_SYMBOLS=OFF
    log "✓ libccd built"
fi

# ────────────────────────────────────────────────────────────────────────────
# 3. FCL — Flexible Collision Library (BSD-3-Clause)
# ────────────────────────────────────────────────────────────────────────────
FCL_XCF="$DEPS_DIR/fcl.xcframework"
if [[ ! -d "$FCL_XCF" ]]; then
    FCL_SRC="$BUILD_DIR/fcl"
    FCL_BLD="$BUILD_DIR/fcl-build"
    FCL_PREFIX="$BUILD_DIR/fcl-install"

    clone_if_needed "$FCL_SRC" \
        "https://github.com/flexible-collision-library/fcl.git" "$FCL_TAG"

    log "Building FCL ${FCL_TAG} …"
    cmake_build "$FCL_SRC" "$FCL_BLD" "$FCL_PREFIX" \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR="$EIGEN_INCLUDE" \
        -Dccd_DIR="$CCD_PREFIX/lib/cmake/ccd" \
        -DFCL_WITH_OCTOMAP=OFF \
        -DBUILD_TESTING=OFF

    FCL_LIB="$(find "$FCL_PREFIX/lib" -name "libfcl*.dylib" | head -1)"
    [[ -n "$FCL_LIB" ]] || die "Could not locate libfcl dylib after build"

    log "Packaging fcl.xcframework …"
    xcodebuild -create-xcframework \
        -library "$FCL_LIB" \
        -headers "$FCL_PREFIX/include" \
        -output "$FCL_XCF"
    log "✓ $FCL_XCF"
else
    warn "fcl.xcframework already exists — skipping (rm -rf $FCL_XCF to rebuild)"
fi

# ────────────────────────────────────────────────────────────────────────────
# 4. OMPL — Open Motion Planning Library (BSD-3-Clause)
# ────────────────────────────────────────────────────────────────────────────
OMPL_XCF="$DEPS_DIR/ompl.xcframework"
if [[ ! -d "$OMPL_XCF" ]]; then
    OMPL_SRC="$BUILD_DIR/ompl"
    OMPL_BLD="$BUILD_DIR/ompl-build"
    OMPL_PREFIX="$BUILD_DIR/ompl-install"

    clone_if_needed "$OMPL_SRC" \
        "https://github.com/ompl/ompl.git" "$OMPL_TAG"

    log "Building OMPL ${OMPL_TAG} (no Python bindings, no demos) …"
    cmake_build "$OMPL_SRC" "$OMPL_BLD" "$OMPL_PREFIX" \
        -DBUILD_SHARED_LIBS=ON \
        -DOMPL_BUILD_DEMOS=OFF \
        -DOMPL_BUILD_TESTS=OFF \
        -DOMPL_BUILD_PYBINDINGS=OFF \
        -DOMPL_REGISTRATION=OFF \
        -DEIGEN3_INCLUDE_DIR="$EIGEN_INCLUDE"

    OMPL_LIB="$(find "$OMPL_PREFIX/lib" -name "libompl*.dylib" | head -1)"
    [[ -n "$OMPL_LIB" ]] || die "Could not locate libompl dylib after build"

    log "Packaging ompl.xcframework …"
    xcodebuild -create-xcframework \
        -library "$OMPL_LIB" \
        -headers "$OMPL_PREFIX/include" \
        -output "$OMPL_XCF"
    log "✓ $OMPL_XCF"
else
    warn "ompl.xcframework already exists — skipping"
fi

# ────────────────────────────────────────────────────────────────────────────
# 5. KDL — Kinematics & Dynamics Library (LGPL-2.1 — dynamic link ONLY)
# ────────────────────────────────────────────────────────────────────────────
KDL_XCF="$DEPS_DIR/kdl.xcframework"
if [[ ! -d "$KDL_XCF" ]]; then
    KDL_REPO="$BUILD_DIR/orocos_kdl_repo"
    KDL_SRC="$KDL_REPO/orocos_kdl"
    KDL_BLD="$BUILD_DIR/kdl-build"
    KDL_PREFIX="$BUILD_DIR/kdl-install"

    clone_if_needed "$KDL_REPO" \
        "https://github.com/orocos/orocos_kinematics_dynamics.git" "$KDL_TAG"

    log "Building KDL ${KDL_TAG} …"
    cmake_build "$KDL_SRC" "$KDL_BLD" "$KDL_PREFIX" \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR="$EIGEN_INCLUDE" \
        -DENABLE_TESTS=OFF

    KDL_LIB="$(find "$KDL_PREFIX/lib" -name "liborocos-kdl*.dylib" | head -1)"
    [[ -n "$KDL_LIB" ]] || die "Could not locate liborocos-kdl dylib after build"

    log "Packaging kdl.xcframework …"
    xcodebuild -create-xcframework \
        -library "$KDL_LIB" \
        -headers "$KDL_PREFIX/include" \
        -output "$KDL_XCF"
    log "✓ $KDL_XCF"
else
    warn "kdl.xcframework already exists — skipping"
fi

# ────────────────────────────────────────────────────────────────────────────
# 6. Regenerate Xcode project now that xcframeworks are present
# ────────────────────────────────────────────────────────────────────────────
log "Regenerating Xcode project (activating xcframework references) …"
cd "$SCRIPT_DIR"

# Uncomment the xcframework lines in project.yml
sed -i '' \
    -e 's|      # - framework: deps/ompl.xcframework|      - framework: deps/ompl.xcframework|' \
    -e 's|      # - framework: deps/fcl.xcframework|      - framework: deps/fcl.xcframework|' \
    -e 's|      # - framework: deps/kdl.xcframework|      - framework: deps/kdl.xcframework|' \
    -e 's|      #   embed: true|      embed: true|' \
    project.yml

xcodegen generate --spec project.yml

# ────────────────────────────────────────────────────────────────────────────
echo ""
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "All dependencies built and project regenerated."
log ""
log "  deps/ompl.xcframework  — BSD-3-Clause"
log "  deps/fcl.xcframework   — BSD-3-Clause"
log "  deps/kdl.xcframework   — LGPL-2.1 (dynamic link only ‼)"
log ""
log "Next: open MoveItMac.xcodeproj in Xcode"
log "  open MoveItMac.xcodeproj"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
