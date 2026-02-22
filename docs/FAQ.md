# rosgraph — Frequently Asked Questions

> **Parent:** [ROSGRAPH.md](ROSGRAPH.md) (technical proposal)

Organized by who's asking. Find your perspective, jump to the
questions that matter to you.

---

## Table of Contents

1. [New ROS Developer](#1-new-ros-developer)
2. [AI-Assisted Developer](#2-ai-assisted-developer)
3. [Engineering Lead / System Integrator / DevOps](#3-engineering-lead--system-integrator--devops)
4. [Safety-Critical Engineer](#4-safety-critical-engineer)
5. [MoveIt / nav2 / Popular Module User](#5-moveit--nav2--popular-module-user)
6. [The Skeptic](#6-the-skeptic)
7. [Package Maintainer / ROS Governance](#7-package-maintainer--ros-governance)
8. [Educator / University Researcher](#8-educator--university-researcher)
9. [Embedded / Resource-Constrained Developer](#9-embedded--resource-constrained-developer)

---

## 1. New ROS Developer

### What problem does rosgraph solve?

ROS 2 doesn't verify that your nodes are wired correctly until
runtime — and often not even then. Type mismatches between publishers
and subscribers fail silently. QoS incompatibilities drop connections
with no error. Parameter renames break launch files with no build
error.

rosgraph catches these at build time. See [PROPOSAL.md §1, "The
Problem, Concretely"](PROPOSAL.md#the-problem-concretely) for four
real-world examples.

### How much do I need to learn?

Write one `interface.yaml` per node (~15 lines for a basic pub/sub
node). Run three commands:

```bash
rosgraph generate .   # generates code
rosgraph lint .       # checks for issues
rosgraph monitor      # watches the running system
```

The YAML schema has IDE autocompletion via JSON Schema. See the [Quick
Start](PROPOSAL.md#quick-start-what-it-looks-like) for a complete
minimal example.

### What do I stop doing when I adopt rosgraph?

- **Stop writing pub/sub boilerplate.** Publisher creation, subscriber
  setup, parameter declaration — all generated from `interface.yaml`.
- **Stop manually syncing parameters between code and launch files.**
  `interface.yaml` is the single source of truth for parameter names,
  types, defaults, and validation ranges.
- **Stop debugging silent QoS mismatches.** `rosgraph lint` catches
  incompatible QoS profiles before you launch.
- **Stop wondering if your launch files reference the right nodes.**
  `rosgraph lint` validates node refs, remappings, and parameter
  overrides.

### Will error messages actually be helpful?

Error quality is a design requirement, not an afterthought. The
architecture follows Ruff's model ([PROPOSAL.md
§10.3](PROPOSAL.md#103-static-analysis-architecture)):

- Every diagnostic includes a rule code (`TOP001`), the location in
  `interface.yaml`, and what's wrong.
- Safe fixes can be auto-applied. Unsafe fixes are flagged but not
  auto-applied.
- SARIF output enables inline annotations in GitHub PRs.
- `--add-noqa` generates suppression comments for existing issues,
  so you can adopt gradually without noise.

### Do I need to learn YAML schema syntax?

Not really. If your editor has the YAML Language Server (most do),
you get autocompletion, inline validation, and hover docs from the
JSON Schema ([PROPOSAL.md §6, G2](PROPOSAL.md#6-feature-list)). Write
a few fields, let the editor fill in the structure.

---

## 2. AI-Assisted Developer

### How does rosgraph work with AI coding tools?

`interface.yaml` is a machine-readable contract — exactly what LLMs
are good at consuming and generating. The `InterfaceDescriptor` IR
([PROPOSAL.md §3.3](PROPOSAL.md#33-the-interfacedescriptor-ir)) is a
JSON blob containing a node's complete API: topics, types, QoS,
parameters, lifecycle state. An AI agent reads this to understand what
a node does, generate implementation code, write tests, or suggest
fixes — without parsing C++ or Python source.

See [PROPOSAL.md §3.13](PROPOSAL.md#313-ai--tooling-integration) for
the full AI integration design.

### Can I use `rosgraph generate` as an agent tool?

Yes. An AI agent writing a ROS node can:
1. Generate `interface.yaml` from a natural language description
2. Run `rosgraph generate .` as a tool call to get type-safe
   scaffolding
3. Write only the business logic into the generated skeleton
4. Run `rosgraph lint .` to verify the graph is correct

This avoids the common failure mode of LLMs hallucinating ROS
boilerplate (wrong QoS defaults, missing component registration,
incorrect parameter declaration).

### Will there be an MCP server?

It's architecturally planned ([PROPOSAL.md
§3.13](PROPOSAL.md#313-ai--tooling-integration)). An MCP server
would expose:
- Graph state (which nodes exist, what they publish/subscribe)
- Lint results (current issues in the workspace)
- Interface schemas (what a specific node expects)
- Resolved topic names (after remapping/namespacing)

This lets Claude Code, Cursor, or Copilot answer "what topics does
the perception pipeline publish?" from structured data, not grep.

### Can an AI generate `interface.yaml` from a description?

Yes — the constrained schema makes this tractable. The schema has
~10 top-level keys with well-defined types. "I need a node that
subscribes to a lidar point cloud, filters it, and publishes the
result" produces a valid `interface.yaml` that `rosgraph generate`
immediately scaffolds.

`rosgraph discover` ([PROPOSAL.md
§3.10](PROPOSAL.md#310-rosgraph-discover--runtime-to-spec-generation))
can also generate `interface.yaml` from a running node, which an LLM
can then refine — adding descriptions, suggesting QoS rationale, and
grouping related interfaces.

### What about IDE / LSP integration?

Phase 1 delivers JSON Schema validation (IDE autocompletion for
`interface.yaml`). A dedicated LSP server would add:
- Hover for message type definitions
- Go-to-definition for `$ref` targets
- Inline diagnostics from `rosgraph lint`
- Cross-file rename support

This benefits both human developers and AI agents operating within
IDE contexts. See [PROPOSAL.md
§3.13](PROPOSAL.md#313-ai--tooling-integration).

---

## 3. Engineering Lead / System Integrator / DevOps

### Who owns an `interface.yaml`?

The node author defines it. Downstream consumers depend on the
installed version in `share/<package>/interfaces/`. Changes are
coordinated via:

- `rosgraph breaking` ([PROPOSAL.md
  §3.9](PROPOSAL.md#39-rosgraph-breaking--breaking-change-detection))
  — automated detection of breaking changes in CI, blocking merges
  that break downstream consumers.
- Installed interfaces — downstream teams depend on published
  interfaces without pulling source code.
- Semantic versioning alignment — breaking = major, dangerous = minor,
  safe = patch.

See [PROPOSAL.md §3.14](PROPOSAL.md#314-scale--fleet-considerations).

### How does this scale to hundreds of packages?

- **Lint performance target:** 100 packages in under 5 seconds
  (Design Principle 7). Analysis is single-pass over the graph model
  with parallel per-package processing and content caching.
- **Multi-workspace analysis:** Installed `interface.yaml` files in
  underlays serve as cached facts. Only your workspace is analyzed,
  not the entire underlay. See [PROPOSAL.md
  §3.12](PROPOSAL.md#312-multi-workspace-analysis).
- **Differential analysis:** `--new-only` reports only issues
  introduced since the base branch. No noise from existing code.
- **Per-package configuration:** Override lint rules per package via
  `rosgraph.toml`.

### I compose nodes from multiple vendors. How does rosgraph help?

`system.yaml` (Layer 2 schema, [PROPOSAL.md
§3.2](PROPOSAL.md#32-schema-layers)) declares the intended system
composition — which nodes, which namespaces, which parameter overrides,
which remappings. `rosgraph lint` validates the composed graph:

- **Type mismatches** across package boundaries (Node A publishes
  `Twist`, Node B subscribes expecting `TwistStamped`)
- **QoS incompatibilities** between a vendor's publisher and your
  subscriber
- **Disconnected subgraphs** — nodes that should be connected but
  aren't due to a namespace or remapping error
- **Invalid remappings** — remaps pointing to nonexistent topics

If a vendor doesn't ship `interface.yaml`, use `rosgraph discover`
([PROPOSAL.md
§3.10](PROPOSAL.md#310-rosgraph-discover--runtime-to-spec-generation))
to generate one from a running instance of the vendor's node. The
discovered spec becomes your integration contract.

### How does rosgraph fit into CI?

rosgraph is CI-first by design (Design Principle 8):

```yaml
# GitHub Actions example
- name: Lint graph
  run: rosgraph lint . --output-format sarif --new-only --base main
  # SARIF output → GitHub Security tab, PR annotations

- name: Check breaking changes
  run: rosgraph breaking --base main
  # Exit code 1 if breaking changes detected

- name: Run contract tests
  run: rosgraph test
  # Schema-driven interface conformance tests
```

Output formats: `text`, `json`, `sarif` (GitHub Security tab),
`github` (Actions annotations), `junit` (test reports). All
configurable via `rosgraph.toml` or `--output-format`. See
[PROPOSAL.md §3.11](PROPOSAL.md#311-configuration).

For brownfield adoption, `--add-noqa` generates inline suppression
comments for all existing issues, creating a clean baseline. You
don't get 500 warnings on your first PR.

### What about the colcon build workflow?

`colcon-rosgraph` (Phase 2) is a thin colcon verb plugin that delegates
to the standalone `rosgraph` binary. It adds `colcon lint`,
`colcon docs`, `colcon discover`, and `colcon breaking` — iterating
packages in dependency order with parallel execution. See [PROPOSAL.md
§3.15](PROPOSAL.md#315-colcon-integration).

Phase 1 works standalone: `rosgraph lint .` in any directory. No
colcon dependency required.

### What about fleet-level monitoring?

`rosgraph monitor` runs per-robot. For fleet-scale observability:

- The Prometheus `/metrics` exporter (M7) enables standard Grafana
  dashboards aggregated across the fleet.
- The `/rosgraph/diff` topic on each robot can be bridged to a
  central system for aggregated drift analysis.
- The architecture uses standard observability patterns (Prometheus,
  structured logs, `/diagnostics`) rather than inventing fleet-specific
  infrastructure.

Runtime performance targets: reconciliation < 500ms for 200 nodes,
< 50MB memory, < 5% CPU at steady state. See [PROPOSAL.md
§3.14](PROPOSAL.md#314-scale--fleet-considerations).

### Can we enforce org-specific conventions?

Yes. `rosgraph.toml` supports per-package rule overrides, custom
naming patterns, and rule selection. The Spectral-inspired YAML-native
rule system ([PROPOSAL.md
§10.3](PROPOSAL.md#103-static-analysis-architecture)) means a
robotics engineer can write custom rules without knowing Rust or C++.

### Does rosgraph handle launch file complexity?

Three strategies, phased by tractability ([PROPOSAL.md
§3.5](PROPOSAL.md#35-rosgraph-lint--static-analysis)):

1. **YAML launch files** — fully parseable, Phase 1
2. **`system.yaml`** — static composition schema, fully analyzable,
   Phase 1
3. **Python launch AST** — pattern matching for `Node()`,
   `LaunchConfiguration()`, etc., Phase 2

Python launch files with complex conditionals, loops, or dynamically
computed node sets can't be fully statically analyzed. `system.yaml`
is the escape hatch for systems that need full analyzability.

---

## 4. Safety-Critical Engineer

### Does rosgraph help with certification?

rosgraph is not a safety tool — it's a development and verification
tool that produces artifacts useful in safety cases. See [PROPOSAL.md
§11](PROPOSAL.md#11-safety--certification) for the full mapping.

Key artifacts:

| rosgraph artifact | Evidence type |
|---|---|
| `interface.yaml` | Software architecture description |
| `rosgraph lint` SARIF output | Static analysis results |
| `rosgraph monitor` logs | Runtime verification evidence |
| `rosgraph test` results | Interface conformance evidence |
| `rosgraph breaking` output | Change impact analysis |

### Which safety standards does this map to?

IEC 61508 (general functional safety), ISO 26262 (automotive),
IEC 62304 (medical), DO-178C (aerospace), ISO 13482 (service robots),
and ISO 21448 / SOTIF. See [PROPOSAL.md
§11.1](PROPOSAL.md#111-relevant-standards) for how rosgraph maps to
each.

### What about behavioral properties?

Phase 1-2 covers structural properties: type matches, QoS
compatibility, graph connectivity. This is a necessary precondition
for behavioral safety — you can't reason about message timing if the
messages aren't connected correctly.

Behavioral analysis (Phase 3+) adds temporal and causal properties,
inspired by HAROS HPL:

```
globally: /emergency_stop causes /motor_disable within 100ms
globally: /heartbeat absent_for 500ms causes /safe_stop
```

See [PROPOSAL.md §11.4](PROPOSAL.md#114-behavioral-properties-future).

### Are monitor alert thresholds configurable?

Yes. The defaults (10s for `NodeMissing`, 30s for `UnexpectedNode`)
are tuned for general robotics. Safety-critical deployments override
them via `rosgraph.toml`:

```toml
[monitor.alerts]
NodeMissing = { grace_period_ms = 1000, severity = "critical" }
TopicMissing = { grace_period_ms = 500, severity = "critical" }
```

See [PROPOSAL.md §11.3](PROPOSAL.md#113-configurable-safety-levels).

### Are there safety-specific lint rules?

Planned for Phase 2-3:

| Rule | Description |
|---|---|
| `SAF001` | Critical subscriber has < N publishers (no redundancy) |
| `SAF002` | Single point of failure in graph topology |
| `SAF003` | Safety-critical node is not lifecycle-managed |
| `TF001` | Declared `frame_id` not published by any node |
| `TF002` | Broken frame chain (no transform path) |

The analyzer architecture supports adding these without changes.
See [PROPOSAL.md §11.5](PROPOSAL.md#115-safety-relevant-lint-rules-future).

### What about determinism and real-time guarantees?

`rosgraph monitor` is an observation tool, not a safety-critical
component. It runs in its own process, does not interfere with the
monitored system, and its failure does not affect the system under
observation. It is not designed to be real-time safe.

For hard real-time requirements, the monitor's output (Prometheus
metrics, diagnostics topics) can be consumed by a separate real-time
safety monitor. rosgraph provides the graph model; the real-time
enforcement layer is a separate concern.

### What about audit trails?

`rosgraph lint` produces SARIF output with timestamps, tool version,
rule versions, and results. This can be stored as CI artifacts for
audit purposes. A dedicated audit log format for `rosgraph monitor`
(continuous verification evidence) is not in Phase 1 but the
structured output (JSON, SARIF) makes it straightforward to add.

---

## 5. MoveIt / nav2 / Popular Module User

### Does rosgraph work with nav2's plugin system?

Yes, via the mixin system ([PROPOSAL.md
§3.2](PROPOSAL.md#32-schema-layers)). Plugins that inject interfaces
into a host node are declared as mixins:

```yaml
# nodes/follow_path/interface.yaml
node:
  name: follow_path
  package: nav2_controller

parameters:
  controller_plugin:
    type: string
    default_value: "dwb_core::DWBLocalPlanner"

mixins:
  - ref: dwb_core/dwb_local_planner   # brings in max_vel_x, etc.
  - ref: nav2_costmap_2d/costmap       # brings in costmap params
```

The host's effective interface = its own declaration + all mixin
interfaces merged. This gives `rosgraph lint` and `rosgraph monitor`
the complete picture.

Mixins are Phase 2 (G15). Phase 1 works for nodes without plugins.

### What happens when I switch plugins (e.g., DWB → MPPI)?

You update the mixin reference in `interface.yaml`. The effective
interface changes at build time, and `rosgraph generate` produces new
scaffolding. This is a build-time concern — `rosgraph lint` validates
the graph with the new plugin's interface.

If the plugin is selected at runtime via parameter, this falls under
"dynamic interfaces" (Design Principle 12) — rosgraph declares the
static portion and `rosgraph monitor` flags unexpected interfaces.

### Does rosgraph validate TF frames?

Planned for Phase 2-3. `TF001` checks that declared `frame_id` values
are published by some node in the graph. `TF002` checks that frame
chains are connected (no broken transform paths). See [PROPOSAL.md
§11.5](PROPOSAL.md#115-safety-relevant-lint-rules-future).

TF is the #1 source of silent bugs in ROS 2 navigation and
manipulation. This is high-value but requires the graph model to
include TF publisher information, which depends on `interface.yaml`
having a `frame_id` annotation.

### What about `generate_parameter_library` compatibility?

Full compatibility is a non-negotiable design principle ([PROPOSAL.md
§2, DP9](PROPOSAL.md#2-design-principles)). The `parameters:` section
of `interface.yaml` IS the `generate_parameter_library` format. A
standalone gen_param_lib YAML file works as-is when placed in
`interface.yaml`. rosgraph delegates to gen_param_lib at build time.
See [PROPOSAL.md §9.2](PROPOSAL.md#92-tool-assessments).

### Can rosgraph lint my existing launch files?

Phase 1 supports YAML launch files (direct parse) and `system.yaml`
(Layer 2 schema). Phase 2 adds Python launch file AST analysis for
standard `launch_ros` patterns — `Node()`, `LaunchConfiguration()`,
`DeclareLaunchArgument()`.

Limitations: Python launch files that use conditionals, loops, or
dynamically computed node sets cannot be fully statically analyzed.
`system.yaml` is the escape hatch for systems that need full static
analyzability. See [PROPOSAL.md
§3.5](PROPOSAL.md#35-rosgraph-lint--static-analysis).

### Does this work with Gazebo / Isaac Sim?

Simulators expose ROS interfaces that look identical to real hardware.
`rosgraph discover` can introspect a simulated system and generate
`interface.yaml`. `rosgraph monitor` can verify that a simulated
system matches the declared graph. `rosgraph lint` doesn't
distinguish between real and simulated — it validates the graph model.

### What about message type changes across ROS distros?

`interface.yaml` references message types by name (e.g.,
`geometry_msgs/msg/Twist`). Message type compatibility across distros
is a ROS infrastructure concern, not a rosgraph concern. rosgraph
validates that publishers and subscribers on the same topic agree on
type — it doesn't validate that the type definition itself is
compatible across distros.

`rosgraph breaking` can detect when a type reference changes between
versions of an `interface.yaml`.

---

## 6. The Skeptic

### I write good tests. Why do I need another YAML file?

Tests catch type mismatches and QoS issues at launch time — after you
wait 30 seconds for the stack to start, watch it fail, read the logs,
and figure out which of 40 nodes has the wrong type. Then you fix it,
rebuild, relaunch, and wait again.

`rosgraph lint` catches the same bugs in under 5 seconds, before
launch, in CI, before anyone else has to debug it. It's the difference
between "tests catch bugs" and "bugs never reach the test phase."

### What's the overhead?

Per node: one `interface.yaml` file (~15-30 lines). Most of it is
information you're already specifying in code (topic names, message
types, QoS settings, parameter names) — `interface.yaml` centralizes
it.

What you get back:
- No pub/sub boilerplate (generated)
- No parameter declaration boilerplate (generated via
  `generate_parameter_library`)
- Pre-launch graph validation
- Runtime graph monitoring
- Auto-generated API documentation

The net line-count change is typically negative for nodes with
parameters.

### What if rosgraph can't express what I need?

Escape hatches:
- **`# rosgraph: noqa: TOP001`** — suppress specific lint rules per
  line.
- **Per-package ignores** — exclude entire packages from specific
  rules via `rosgraph.toml`.
- **Undeclared interfaces** — if your code creates publishers that
  aren't in `interface.yaml`, the code still works. `rosgraph monitor`
  flags them as `UnexpectedTopic` (a warning, not an error).
- **Composition pattern** — generated code holds a `rclcpp::Node`
  (has-a), not inherits from it. You always have access to the
  underlying node for anything the schema can't express.

See [PROPOSAL.md §12](PROPOSAL.md#12-scope--limitations) for the full
limitations discussion.

### Does code generation add runtime overhead?

The composition pattern (has-a Node) adds one level of indirection
compared to direct inheritance. This is a pointer dereference — single
nanoseconds. The generated pub/sub wrappers are thin forwarding calls.
No virtual dispatch is added that wouldn't already exist in the ROS
client library.

The parameter validation code (from `generate_parameter_library`) runs
at parameter-set time, not in the hot path.

### What happens when only my package has an `interface.yaml`?

You still get:
- **Code generation** — less boilerplate in your node
- **Parameter validation** — runtime type and range checking
- **Self-documentation** — your node's API is machine-readable

Cross-package value (type mismatch detection, QoS compatibility
checking, contract testing) grows with adoption. `rosgraph discover`
lets you generate specs for neighboring packages from a running
system, bootstrapping the cross-package graph incrementally.

### This proposal has 51 features. Is this realistic?

Phase 1 ([PROPOSAL.md §4](PROPOSAL.md#4-phasing)) is the commitment:
~12 features covering core schema, basic code generation, and
highest-value lint and monitor rules. Later phases are contingent on
adoption.

The tool builds on existing work — cake for code generation,
`generate_parameter_library` for parameters, `graph-monitor` message
definitions for runtime. Phase 1 is stabilizing and unifying existing
pieces, not building from scratch.

### Won't the spec just drift from reality like NoDL?

NoDL died because it was a pure description format — no code
generation. Maintaining a spec that doesn't produce anything is
thankless work.

`interface.yaml` generates code. If you change the spec, the generated
code changes. If you change the code without changing the spec,
`rosgraph monitor` flags the discrepancy at runtime. The two-way
binding (codegen + runtime monitoring) is what prevents the drift
that killed NoDL.

The honest limitation: business logic is hand-written. If a developer
adds an undeclared publisher inside a callback, `rosgraph lint` won't
catch it at build time. `rosgraph monitor` catches it at runtime as
`UnexpectedTopic`. See [PROPOSAL.md
§12](PROPOSAL.md#12-scope--limitations).

### When should I NOT use rosgraph?

- **Quick prototyping** — single throwaway node, not worth the file.
- **Single-node packages** — minimal lint value, though codegen may
  still save boilerplate.
- **Highly dynamic interfaces** — nodes that create/destroy publishers
  at runtime based on conditions can't be fully declared.

See [PROPOSAL.md §12, "When Not to Use
rosgraph"](PROPOSAL.md#when-not-to-use-rosgraph).

---

## 7. Package Maintainer / ROS Governance

### What does rosgraph mean for my package?

If you maintain a ROS 2 package, `interface.yaml` is a machine-readable
contract for your node's public API — topics, services, actions,
parameters, QoS. It replaces the informal contract currently scattered
across READMEs, launch file comments, and source code.

For consumers of your package, this means:
- **API discoverability.** `rosgraph docs` auto-generates browsable API
  reference from your `interface.yaml`. No more stale READMEs.
- **Breaking change visibility.** `rosgraph breaking` classifies
  interface changes as breaking/dangerous/safe, giving downstream users
  clear upgrade guidance. See [PROPOSAL.md
  §3.9](PROPOSAL.md#39-rosgraph-breaking--breaking-change-detection).
- **Contract testing.** Downstream packages can run `rosgraph test`
  against your declared interface to verify compatibility. See
  [PROPOSAL.md §3.7](PROPOSAL.md#37-rosgraph-test--contract-testing).

### Do I have to adopt rosgraph to be compatible with it?

No. Packages without `interface.yaml` are skipped, not errored (Design
Principle 6). Downstream users can run `rosgraph discover` against your
running node to generate a spec for their own use. Your package doesn't
need to ship `interface.yaml` for others to benefit — though shipping
one is much better, since discovered specs require human review and may
miss QoS details.

### How does this affect my release process?

`rosgraph breaking` runs in CI comparing the current `interface.yaml`
against the previous release. Breaking changes block the merge unless
explicitly acknowledged. This is opt-in per package via `rosgraph.toml`
and maps to semantic versioning: breaking = major, dangerous = minor,
safe = patch. See [PROPOSAL.md
§3.14](PROPOSAL.md#314-scale--fleet-considerations).

### What about packages with plugin systems?

If your package exposes a plugin API (like nav2's controller plugins),
the mixin system (Phase 2, G15) lets plugin authors declare the
interfaces they inject into the host node. The host's effective
interface is the merge of its own declaration plus all mixin fragments.
See [PROPOSAL.md §3.2](PROPOSAL.md#32-schema-layers).

Until mixins ship in Phase 2, the host node's `interface.yaml` covers
its own direct interfaces. Plugins that add extra topics/parameters
are flagged by `rosgraph monitor` as unexpected — visible but not
validated.

### What's the adoption path toward `ros_core`?

Deliberately incremental ([PROPOSAL.md §4, "Adoption
Path"](PROPOSAL.md#adoption-path)):

1. **`ros-tooling` organization** — institutional backing, CI
   infrastructure, release process. graph-monitor already lives here.
2. **REP for `interface.yaml` schema** — formalizes the declaration
   format as a community standard, independent of the rosgraph tool.
3. **docs.ros.org tutorial integration** — if "write your first node"
   uses `interface.yaml`, every new ROS developer learns it from day
   one. This is the highest-leverage adoption path.
4. **`ros_core` proposal** — after demonstrated adoption across
   multiple distros.

### Why not extend existing tools instead?

Each existing tool covers one capability but none covers the full
scope. The gap analysis ([PROPOSAL.md
§9.3](PROPOSAL.md#93-gap-analysis)) shows five major gaps: graph diff,
graph linting, QoS static analysis, behavioral properties, and CI graph
validation. No single existing tool can be extended to fill all five.

rosgraph builds on existing work where possible:
- `generate_parameter_library` for parameters (used as-is)
- `rosgraph_monitor_msgs` for runtime message definitions (adopted)
- cake's design decisions for code generation (validated)
- HAROS's metamodel for the graph model (adapted)

### What's the maintenance burden?

Phase 1 is ~12 features covering core schema, basic codegen, and
highest-value lint/monitor rules. The design minimizes ongoing
maintenance:

- **Schema versioning** (G14) — `schema_version` field with migration
  tooling prevents breaking changes to `interface.yaml` format.
- **IR-based plugin protocol** — code generation plugins are standalone
  executables, independently maintained.
- **Analyzer DAG** — lint rules are isolated, independently testable
  values (not subclasses). Adding or removing a rule doesn't affect
  others.

The risk factor: this is a new tool, not an extension of something with
existing momentum. It requires sustained contributor commitment.

### How does this interact with the ROS 2 type system?

rosgraph references existing `.msg`, `.srv`, and `.action` types — it
doesn't replace them (Design Principle 9). `interface.yaml` declares
which types a node uses; `rosidl` still defines the types themselves.

The graph model ([PROPOSAL.md §3.1](PROPOSAL.md#31-the-graph-model))
includes a `MessageTypeDB` that resolves type references to their
definitions for compatibility checking. This uses the existing
`rosidl` output — rosgraph doesn't parse `.msg` files directly.

### What about governance and community standards?

The REP process is the standard mechanism for formalizing ROS community
standards. A REP for the `interface.yaml` schema would:

- Define the YAML schema specification independent of the rosgraph tool
- Allow alternative implementations (someone could build a different
  tool that consumes the same schema)
- Provide a formal review process for schema changes
- Signal community endorsement

The REP is Step 2 of the adoption path — after the tool has proven
itself in `ros-tooling` with real users.

### What's the risk if this doesn't get adopted?

The worst case: rosgraph becomes another single-maintainer tool in the
ecosystem (like cake and breadcrumb today). The mitigation strategy:

- **`ros-tooling` hosting** — institutional backing reduces bus factor
- **REP-based schema** — the schema outlives the tool if it becomes a
  standard
- **`generate_parameter_library` compatibility** — the parameters
  portion works with the most mature tool in the space, regardless of
  rosgraph's fate
- **Standalone value** — even without ecosystem adoption, a single team
  gets code generation and parameter validation from day one

---

## 8. Educator / University Researcher

### Can I use rosgraph for teaching ROS 2?

Yes, and this is one of the highest-leverage adoption paths. The Quick
Start ([PROPOSAL.md §1](PROPOSAL.md#quick-start-what-it-looks-like))
shows a complete workflow in 3 commands:

```bash
rosgraph generate .   # generates node scaffolding
rosgraph lint .       # checks for issues
rosgraph monitor      # watches the running system
```

For teaching, `interface.yaml` forces students to think about their
node's API before writing implementation code — topics, types, QoS,
parameters. This is better pedagogy than the current approach of
copy-pasting publisher boilerplate and tweaking it.

### Does this lower the barrier for students?

Significantly. A student writes ~15 lines of YAML declaring what their
node does, runs `rosgraph generate`, and gets a working scaffold with
type-safe publishers, subscribers, and validated parameters. They write
only the business logic. No boilerplate, no silent type mismatches, no
mysterious QoS failures.

Error messages are designed to be helpful — rule codes, file locations,
clear descriptions of what's wrong and how to fix it. See [PROPOSAL.md
§10.3](PROPOSAL.md#103-static-analysis-architecture).

### How does rosgraph relate to HAROS?

HAROS ([PROPOSAL.md §10.6](PROPOSAL.md#106-ros-domain-prior-art-haros))
was the prior art for graph analysis in ROS — built at the University
of Minho (2016–2021). rosgraph borrows HAROS's metamodel and HPL
property language concepts, but differs fundamentally:

- **HAROS extracted interfaces from source code.** rosgraph uses
  explicit declarations (`interface.yaml`). Declarations are simpler,
  more reliable, and enable code generation.
- **HAROS was ROS 1 only.** rosgraph is built for ROS 2 concepts:
  QoS, lifecycle, components, actions, DDS discovery.
- **HAROS died because extraction broke.** catkin → ament, rospack →
  colcon, XML launch → Python launch. Declaration-based tools don't
  break when the build system changes.

### Can I use rosgraph for research on ROS system verification?

The graph model ([PROPOSAL.md §3.1](PROPOSAL.md#31-the-graph-model))
is a structured representation of the ROS computation graph — nodes,
topics, services, actions, parameters, QoS, connections. It's
exportable as JSON via the `InterfaceDescriptor` IR ([PROPOSAL.md
§3.3](PROPOSAL.md#33-the-interfacedescriptor-ir)).

Research opportunities:
- **Formal verification.** The graph model is a natural input for model
  checkers. Behavioral properties (Phase 3+, [PROPOSAL.md
  §11.4](PROPOSAL.md#114-behavioral-properties-future)) enable temporal
  logic specifications.
- **Static analysis.** The analyzer DAG architecture ([PROPOSAL.md
  §3.5](PROPOSAL.md#35-rosgraph-lint--static-analysis)) supports custom
  analysis passes without modifying core code.
- **Runtime monitoring.** The declared-vs-observed diff ([PROPOSAL.md
  §3.6](PROPOSAL.md#36-rosgraph-monitor--runtime-reconciliation)) is a
  rich data source for anomaly detection research.
- **ROS ecosystem studies.** Interface coverage, graph topology
  patterns, common QoS configurations — all extractable from
  `interface.yaml` files across the ecosystem.

### What about publishing results that use rosgraph?

The tool is open-source (planned for `ros-tooling` organization). The
SARIF and JSON output formats produce structured, reproducible results
suitable for academic publication. The graph model provides a formal
vocabulary for describing ROS system architectures.

---

## 9. Embedded / Resource-Constrained Developer

### Does rosgraph add runtime overhead to my nodes?

The generated code uses a composition pattern (has-a `Node`, not is-a
`Node`). This adds one pointer indirection — single nanoseconds. The
generated pub/sub wrappers are thin forwarding calls. No virtual
dispatch is added beyond what the ROS client library already uses.

Parameter validation (via `generate_parameter_library`) runs at
parameter-set time, not in the hot path. See [PROPOSAL.md §3.4,
"Design decisions"](PROPOSAL.md#34-rosgraph-generate--code-generation).

### Does `rosgraph monitor` run on the robot?

Yes, but it's optional. `rosgraph monitor` is a separate process — it
doesn't instrument or modify your nodes. If your platform can't spare
the resources, don't run it. You still get full value from build-time
tools (`rosgraph generate`, `rosgraph lint`).

Runtime targets for `rosgraph monitor` ([PROPOSAL.md
§3.14](PROPOSAL.md#314-scale--fleet-considerations)):
- Memory: < 50MB resident
- CPU: < 5% of one core at steady-state (5s scrape interval)
- No additional DDS traffic beyond standard discovery

For very constrained platforms, run `rosgraph monitor` off-board
(e.g., on a companion computer) observing the same DDS domain.

### Does rosgraph work with micro-ROS?

micro-ROS nodes communicate via the standard DDS/XRCE-DDS bridge.
`rosgraph discover` and `rosgraph monitor` observe them through the
bridge like any other node. `interface.yaml` declarations work for
micro-ROS nodes — the schema is language-agnostic.

Code generation for micro-ROS C is not in Phase 1. The IR-based plugin
architecture ([PROPOSAL.md
§3.3](PROPOSAL.md#33-the-interfacedescriptor-ir)) supports adding a
micro-ROS code generation plugin without changes to the core tool.

### What about real-time constraints?

`rosgraph monitor` is not real-time safe — it's an observation tool
running in its own process. It does not interfere with the monitored
system, and its failure does not affect the system under observation.

For hard real-time requirements, the monitor's Prometheus metrics and
diagnostics topics can be consumed by a separate real-time safety
monitor. rosgraph provides the graph model; real-time enforcement is a
separate concern. See [PROPOSAL.md
§11](PROPOSAL.md#11-safety--certification).

### Does the build toolchain add cross-compilation complexity?

`rosgraph generate` runs at build time on the host, producing standard
C++ and Python source files. These are compiled by the normal
cross-compilation toolchain (`colcon build --cmake-args
-DCMAKE_TOOLCHAIN_FILE=...`). rosgraph itself doesn't need to run on
the target — it's a host-side tool, like `cmake` or `protoc`.
