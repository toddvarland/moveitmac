.PHONY: setup deps gen open test-urdf clean-deps help

# ── Default ──────────────────────────────────────────────────────────────────
help:
	@echo ""
	@echo "MoveIt Mac — build targets"
	@echo ""
	@echo "  make setup      Build C++ deps + generate Xcode project (first-time setup)"
	@echo "  make deps       Build/rebuild C++ xcframeworks only"
	@echo "  make gen        Regenerate MoveItMac.xcodeproj from project.yml"
	@echo "  make open       Run setup then open in Xcode"
	@echo "  make test-urdf  Run URDFKit unit tests via swift test"
	@echo "  make clean-deps Remove deps/ and .deps_build/ (forces full rebuild)"
	@echo ""

# ── First-time setup ─────────────────────────────────────────────────────────
setup: deps gen

# ── Build C++ xcframeworks ───────────────────────────────────────────────────
deps:
	@bash build_deps.sh

# ── Generate .xcodeproj ──────────────────────────────────────────────────────
gen:
	@xcodegen generate --spec project.yml

# ── Open in Xcode ────────────────────────────────────────────────────────────
open: setup
	@open MoveItMac.xcodeproj

# ── Run URDFKit unit tests (no Xcode needed) ─────────────────────────────────
test-urdf:
	@swift test --package-path Packages/URDFKit

# ── Remove built deps (forces full rebuild on next `make deps`) ──────────────
clean-deps:
	@echo "Removing deps/ and .deps_build/ …"
	@rm -rf deps .deps_build
	@echo "Run 'make setup' to rebuild."
