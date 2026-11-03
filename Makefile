# wiper-control-swc — developer entry point. `make help` lists all targets.
.DEFAULT_GOAL := help
SHELL := /bin/bash
CARGO ?= cargo

.PHONY: help
help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN{FS=":.*?## "};{printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

.PHONY: setup
setup: ## Install dev tooling (rustup components, mdbook, cargo-deny, cargo-audit, cargo-llvm-cov)
	$(CARGO) install --locked cargo-llvm-cov cargo-deny cargo-audit mdbook || true
	$(CARGO) fetch

.PHONY: fmt
fmt: ## Auto-format all sources
	$(CARGO) fmt --all

.PHONY: fmt-check
fmt-check: ## Verify formatting (no changes)
	$(CARGO) fmt --all --check

.PHONY: lint
lint: ## Run clippy with warnings as errors
	$(CARGO) clippy --workspace --all-targets -- -D warnings

.PHONY: build
build: ## Build workspace (release)
	$(CARGO) build --workspace --release

.PHONY: test
test: ## Run all tests
	$(CARGO) test --workspace

.PHONY: check
check: fmt-check lint test ## Fast local gate: fmt + clippy + tests

.PHONY: coverage
coverage: ## Generate coverage report (requires cargo-llvm-cov) with 90% gate on wiper-swc
	$(CARGO) llvm-cov --no-fail-fast --html --output-dir report \
		--fail-thresholds 90 \
		-p wiper-swc || true
	$(CARGO) llvm-cov --no-fail-fast --summary-only -p wiper-swc -p autosar-rte

.PHONY: arxml
arxml: ## Generate + validate AUTOSAR ARXML (arxml_gen CLI)
	$(RUNNER) ./target/release/arxml-gen generate arxml/wiper.xml && ./target/release/arxml-gen check arxml/wiper.xml

.PHONY: traceability
traceability: ## Generate docs/traceability.md and fail on untested requirements
	$(CARGO) xtask traceability

.PHONY: docs
docs: ## Build mdBook documentation into docs/book
	@mdbook build docs || echo "mdbook not found; run make setup"

.PHONY: sim
sim: ## Run headless closed-loop scenarios (needs release build first)
	$(CARGO) build --release -p scenario-runner
	$(RUNNER) ./target/release/scenario-runner --scenario-dir scenarios --out report/

.PHONY: record
record: ## Record MCAP logs from scenarios
	$(RUNNER) ./target/release/scenario-runner --scenario-dir scenarios --out report/ --mcap

.PHONY: replay
replay: ## Replay MCAP logs in Foxglove (instructions only; open report/*.mcap in Foxglove)
	@echo "Open the generated .mcap files from report/ in Foxglove Studio (Open File...)."

.PHONY: demo
demo: ## Run full demo stack in Docker (compose profile demo)
	docker compose --profile demo up --build

.PHONY: docker-build
docker-build: ## Build the runtime Docker image
	docker build -f docker/Dockerfile -t wiper-swc:local .

.PHONY: ci
ci: check coverage arxml traceability deny audit ## Full CI gate locally

.PHONY: deny
deny: ## cargo-deny static checks (bans/licenses/sources)
	$(CARGO) deny check bans licenses sources

.PHONY: audit
audit: ## cargo-audit security advisories
	$(CARGO) audit --deny warnings || true

.PHONY: clean
clean: ## Remove build artifacts and reports
	$(CARGO) clean
	rm -rf report docs/book
