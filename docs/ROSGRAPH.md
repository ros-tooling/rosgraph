# rosgraph — Technical Proposal

> **Status:** Proposal
> **Date:** 2026-02-22
> **Parent:** [MANIFESTO.md](MANIFESTO.md) (direction)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Design Principles](#2-design-principles)
3. [Architecture](#3-architecture)
4. [Phasing](#4-phasing)
5. [Language Choice](#5-language-choice)
6. [Feature List](#6-feature-list)
7. [Lint Rule Codes](#7-lint-rule-codes)
8. [Monitor Alert Rules](#8-monitor-alert-rules)
9. [Existing ROS 2 Ecosystem](#9-existing-ros-2-ecosystem)
10. [Prior Art](#10-prior-art)
11. [Safety & Certification](#11-safety--certification)
12. [Scope & Limitations](#12-scope--limitations)
13. [Resolved Questions](#13-resolved-questions)

---

## 1. Executive Summary

ROS 2 has no production-ready tool for verifying that a running system
matches its declared architecture, no standard schema for declaring node
interfaces, and no unified CLI for graph analysis. The ecosystem is
fragmented across single-purpose tools with overlapping scope and bus
factors of one.

| Category | Capability | Current tool | Status |
|---|---|---|---|
| **Schema** | Node interface declaration | cake / nodl / gen_param_lib | cake early; nodl dead; gpl params-only |
| **Codegen** | Static graph from launch files | breadcrumb + clingwrap | Early-stage, solo dev |
| **Runtime** | Runtime graph monitoring | graph-monitor | Mid-stage, institutional |
| **Runtime** | Runtime tracing | ros2_tracing | Mature, production |
| **Runtime** | Latency analysis | CARET | Mature, Tier IV |
| **Runtime** | Graph visualisation | Foxglove, Dear RosNodeViewer | Mature but live-only |
| **Runtime** | **Graph diff (expected vs. actual)** | **Nothing** | **Major gap** |
| **Static** | **Graph linting (pre-launch)** | **Nothing** | **Major gap** |
| **Static** | **QoS static analysis** | breadcrumb (partial) | Early-stage |
| **Static** | **CI graph validation** | **Nothing** | **Major gap** |
| **Docs** | **Node API documentation** | **Nothing** (hand-written only) | **Major gap** |
| — | **Behavioural properties** | **Nothing** (HPL was ROS 1) | **Major gap** |

### The Problem, Concretely

Today in ROS 2:

- Node A publishes `/cmd_vel` as `Twist`. Node B subscribes to
  `/cmd_vel` as `String`. You discover this at runtime — or don't,
  because the subscriber silently receives nothing.
- A publisher uses `BEST_EFFORT` QoS, a subscriber uses `RELIABLE`.
  DDS refuses the connection. A warning is logged but easy to miss in
  a busy console. The subscriber just never gets messages.
- A node crashes mid-deployment. The rest of the system keeps running.
  Nobody knows until a customer reports a failure 20 minutes later.
- You rename a parameter. Three launch files reference the old name.
  `colcon build` succeeds. The system launches. The parameter silently
  takes its default value.

These are real, common bugs in production ROS 2 systems. rosgraph
catches all four — the first two at build time (`rosgraph lint`), the
third at runtime (`rosgraph monitor`), the fourth at lint time.

This document proposes **`rosgraph`** — a single tool with subcommands
covering the four goals of the ROSGraph Working Group:

```
rosgraph
├── rosgraph generate    (Goal 2: spec → code)
├── rosgraph lint        (Goal 4: static graph analysis)
├── rosgraph monitor     (Goal 3: runtime reconciliation)
├── rosgraph test        (Goal 3: contract testing)
├── rosgraph docs        (documentation generation)
├── rosgraph breaking    (breaking change detection)
└── rosgraph discover    (runtime → spec, brownfield adoption)
```

Three key insights drive the design:

1. **The ROS computation graph is not source code — it is a typed,
   directed graph with QoS-annotated edges.** Analysis tools should
   operate on a graph model, not on ASTs. Source code parsing is a
   loader that feeds the model, not the analysis target.

2. **Goals 3–4 are schema conformance problems** ("does reality match
   the spec?"), not traditional program analysis. Once you have a
   machine-readable spec (`interface.yaml`), verification falls out
   naturally — the same pattern as `buf lint`, Pact contract tests,
   and Kubernetes reconciliation.

3. **A declaration without code generation is a non-starter.** NoDL
   proved this. The schema must generate code, documentation, and
   validation to stay in sync with reality. `interface.yaml` is
   simultaneously the source for code generation, the lint target for
   static analysis, the contract for runtime verification, and the
   reference for documentation.

### Quick Start (What It Looks Like)

A minimal `interface.yaml`:

```yaml
schema_version: "1.0"
node:
  name: talker
  package: demo_pkg

publishers:
  - topic: ~/chatter
    type: std_msgs/msg/String
    qos: { reliability: RELIABLE, depth: 10 }

parameters:
  publish_rate:
    type: double
    default_value: 1.0
    description: "Publishing rate in Hz"
    validation:
      bounds<>: [0.1, 100.0]
```

What you get:

```bash
rosgraph generate .   # → C++ header, Python module, parameter validation
rosgraph lint .       # → "no issues" or "TOP001: type mismatch on /cmd_vel"
rosgraph monitor      # → live diff: declared graph vs. running system
```

The generated code gives you a typed context struct with publishers,
subscribers, and validated parameters — no boilerplate. You write
business logic; rosgraph generates the wiring.

---

## 2. Design Principles

### Core Philosophy

1. **The graph is the program.** Analysis operates on the typed,
   QoS-annotated computation graph — not source code ASTs. Source
   parsing is a loader that feeds the model, not the analysis target.

2. **Declare first, verify always.** `interface.yaml` is the single
   source of truth. Code generation, static analysis, and runtime
   monitoring all verify against the declaration.

3. **One schema, many consumers.** The same `interface.yaml` drives
   code generation, documentation, linting, monitoring, contract
   testing, and security policy generation.

4. **One tool, not ten.** `rosgraph` with subcommands replaces
   fragmented single-purpose tools. One CLI, one config, one output
   format.

### Developer Experience

5. **Zero-config value, progressive disclosure.** Given
   `interface.yaml` files, the default rules catch real bugs (type
   mismatches, QoS incompatibilities) with no additional configuration.
   A minimal 10-line `interface.yaml` produces a working node;
   lifecycle, mixins, and parameterized QoS are opt-in.

6. **Brownfield first, gradual adoption.** `rosgraph discover`
   generates specs from running nodes. `--add-noqa` suppresses existing
   issues. Packages without `interface.yaml` are skipped, not errored.

7. **Speed is a feature.** An architectural property, not an
   afterthought. Target: lint a 100-package workspace in under 5
   seconds.

8. **Backward compatibility is non-negotiable.** Existing
   `generate_parameter_library` YAML works as-is inside `parameters:`.
   Existing `.msg`/`.srv`/`.action` files are referenced, not replaced.

### Verification & CI

9. **CI-first.** SARIF output, GitHub annotations, exit codes, and
   differential analysis are primary design targets.

10. **Validation at every stage.** Author time: JSON Schema. Build
    time: structural + semantic. Launch time: declared vs. configured.
    Runtime: declared vs. observed.

11. **Correctness rules are errors; style rules are warnings.** Type
    mismatches and QoS incompatibilities fail CI. Naming conventions
    warn.

### Scope

12. **Declared interfaces are the primary target.** The schema
    describes the *intended* interface — the same boundary drawn by
    Protobuf, AsyncAPI, Smithy, and OpenAPI. For partially dynamic
    nodes (e.g., nav2 plugin hosts), worst-case bounds can be declared
    with `optional: true`; `rosgraph monitor` validates these at
    runtime and flags truly undeclared interfaces as `UnexpectedTopic`.

13. **Structural first, behavioural later.** Phase 1–2: type matches,
    QoS compatibility, graph connectivity — the foundation that
    safety-critical systems (ISO 26262, IEC 61508) require as evidence.
    Behavioural properties (temporal/causal, e.g. "/e_stop causes
    /motor_disable within 100ms") are Phase 3+, drawing on prior art
    from HAROS HPL and runtime verification tools like STL/MTL
    monitors. The structural graph model is designed to extend to
    behavioural annotations without schema redesign.

---

## 3. Architecture

One tool. One graph model. Four capabilities.

```
                         ┌──────────────────────┐
                         │    Graph Model        │
                         │  (shared library)     │
                         │                       │
                         │  Nodes, Topics,       │
                         │  Services, Actions,   │
                         │  Parameters, QoS,     │
                         │  Connections           │
                         └───────┬──────┬────────┘
                                 │      │
                    ┌────────────┘      └───────────────┐
                    │                                   │
          ┌─────────▼──────────┐            ┌──────────▼───────────┐
          │  Build-time tools  │            │  Runtime tools        │
          │                    │            │                       │
          │  rosgraph generate │            │  rosgraph monitor     │
          │  rosgraph lint     │            │  rosgraph test        │
          │  rosgraph docs     │            │  rosgraph discover    │
          │  rosgraph breaking │            │                       │
          └────────────────────┘            └───────────────────────┘
```

### 3.1 The Graph Model

A language-agnostic representation of the ROS computation graph. Every
loader produces it; every analyzer consumes it.

```
ComputationGraph
├── nodes: [NodeInterface]
│   ├── name, namespace, package, executable
│   ├── publishers:     [{topic, msg_type, qos}]
│   ├── subscribers:    [{topic, msg_type, qos}]
│   ├── services:       [{name, srv_type}]
│   ├── clients:        [{name, srv_type}]
│   ├── action_servers: [{name, action_type}]
│   ├── action_clients: [{name, action_type}]
│   ├── parameters:     [{name, type, default, validators}]
│   └── lifecycle_state: str | None
├── topics: [TopicInfo]
│   ├── name, msg_type
│   ├── publishers:  [NodeRef]
│   ├── subscribers: [NodeRef]
│   └── qos_profiles: [QoSProfile]
├── services: [ServiceInfo]
├── actions: [ActionInfo]
└── connections: [Connection]
    ├── source: NodeRef
    ├── target: NodeRef
    ├── channel: TopicRef | ServiceRef | ActionRef
    └── qos_compatible: bool
```

### 3.2 Schema Layers

Three schema levels, each building on the previous:

**Layer 1 — Node Interface Schema** (per-node declaration)

```yaml
# interface.yaml
schema_version: "1.0"

node:
  name: lidar_processor
  package: perception_pkg
  lifecycle: managed              # managed | unmanaged (default)

parameters:
  # Exact generate_parameter_library format (backward-compatible)
  voxel_size:
    type: double
    default_value: 0.05
    description: "Voxel grid filter leaf size (meters)"
    validation:
      bounds<>: [0.01, 1.0]
    read_only: false
  robot_frame:
    type: string
    default_value: "base_link"
    read_only: true

publishers:
  - topic: ~/filtered_points
    type: sensor_msgs/msg/PointCloud2
    qos:
      history: 5
      reliability: RELIABLE
      durability: TRANSIENT_LOCAL
    description: "Filtered and downsampled point cloud"

subscribers:
  - topic: ~/raw_points
    type: sensor_msgs/msg/PointCloud2
    qos:
      history: 1
      reliability: BEST_EFFORT
    description: "Raw point cloud from lidar driver"

services:
  - name: ~/set_filter_params
    type: perception_msgs/srv/SetFilterParams

actions:
  - name: ~/process_scan
    type: perception_msgs/action/ProcessScan

timers:
  - name: process_timer
    period_ms: 100
    description: "Main processing loop"
```

**Layer 2 — Composed System Schema** (launch-level declaration)

```yaml
# system.yaml
schema_version: "1.0"
name: perception_pipeline

nodes:
  - ref: perception_pkg/lidar_processor
    namespace: /robot1
    parameters:
      voxel_size: 0.1
    remappings:
      ~/raw_points: /lidar/points

  - ref: perception_pkg/object_detector
    namespace: /robot1

connections:                    # Explicit wiring (optional, for validation)
  - from: lidar_processor/~/filtered_points
    to: object_detector/~/input_cloud
```

**Mixins — Composable Interface Fragments** (G15, Phase 2)

Plugins that inject interfaces into a host node (e.g., nav2 controller
plugins adding parameters and topics via the node handle) are declared
via `mixins:`. Each mixin is itself an `interface.yaml` fragment
declaring the topics, parameters, and services it adds. The host node's
effective interface is the merge of its own declaration plus all mixins.

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
  - ref: dwb_core/dwb_local_planner   # brings in max_vel_x, min_vel_y, etc.
  - ref: nav2_costmap_2d/costmap       # brings in costmap params + topics
```

This pattern (borrowed from Smithy's mixin concept) gives `rosgraph
lint` and `rosgraph monitor` the complete interface picture without
requiring the host node to redeclare everything its plugins add.
Requires the `$ref` / fragment system (G15) as a prerequisite.

**Layer 3 — Observation Schema** (runtime-observed state)

```yaml
# observed.yaml (auto-generated from running system)
node:
  name: lidar_processor
  package: perception_pkg
  pid: 12345
  state: active                 # lifecycle state if managed

publishers:
  - topic: /robot1/lidar_processor/filtered_points
    type: sensor_msgs/msg/PointCloud2
    qos:
      reliability: RELIABLE
      durability: TRANSIENT_LOCAL
      depth: 5
    stats:
      message_count: 14523
      frequency_hz: 9.98
      subscribers_matched: 2

# ... subscribers, services, actions, parameters with actual values
```

### 3.3 The InterfaceDescriptor (IR)

The parsed, validated, fully-resolved representation of a node's
interface. Serializable as JSON for plugin communication:

```json
{
  "schema_version": "1.0",
  "node": {
    "name": "lidar_processor",
    "package": "perception_pkg",
    "lifecycle": "managed"
  },
  "parameters": [
    {
      "name": "voxel_size",
      "type": "double",
      "default_value": 0.05,
      "description": "Voxel grid filter leaf size (meters)",
      "validation": { "bounds": [0.01, 1.0] },
      "read_only": false
    }
  ],
  "publishers": [
    {
      "topic": "~/filtered_points",
      "resolved_topic": "/robot1/lidar_processor/filtered_points",
      "message_type": "sensor_msgs/msg/PointCloud2",
      "qos": { "history": 5, "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL" },
      "description": "Filtered and downsampled point cloud"
    }
  ]
}
```

Plugins receive this IR via stdin (or file path) and produce generated
files.

### 3.4 `rosgraph generate` — Code Generation

Translates `interface.yaml` into working node implementations.

```
┌─────────────────────────────────────────┐
│         interface.yaml (per node)        │
└────────────────┬────────────────────────┘
                 │
┌────────────────▼────────────────────────┐
│          Parser / Validator              │
│  1. YAML parse                          │
│  2. JSON Schema validation (structural) │
│  3. Semantic validation (type refs, QoS)│
│  4. Produce InterfaceDescriptor (IR)    │
└────────────────┬────────────────────────┘
                 │
      ┌──────────┼──────────────────┐
      │          │                  │
┌─────▼─────┐ ┌─▼──────────┐ ┌────▼──────┐
│ C++ Plugin│ │Python Plugin│ │Docs Plugin│
│           │ │             │ │           │
│ - header  │ │ - module    │ │ - API ref │
│ - reg.cpp │ │ - params    │ │ - graph   │
│ - params  │ │ - __init__  │ │   fragment│
└───────────┘ └─────────────┘ └───────────┘
```

**Build integration:**

```cmake
cmake_minimum_required(VERSION 3.22)
project(perception_pkg)
find_package(rosgraph REQUIRED)
rosgraph_auto_package()
```

Under the hood, `rosgraph_auto_package()`:
1. Scans `nodes/` for subdirectories with `interface.yaml`
2. Validates each `interface.yaml` (structural + semantic)
3. Invokes C++ plugin → header, registration, params YAML
4. Delegates to `generate_parameter_library()` for parameters
5. Compiles and links
6. Installs interface YAMLs to `share/<package>/interfaces/`

**Design decisions:**
- **Composition over inheritance.** Generated code holds a
  `rclcpp::Node` (has-a), not inherits from it. Context struct is a
  flat aggregation of generated components plus user state.
- **`generate_parameter_library` as backend.** Uses the existing,
  widely-adopted parameter library rather than reimplementing.
- **Convention-over-configuration.** Directory layout (`nodes/`,
  `interfaces/`, `launch/`, `config/`) determines behavior.

### 3.5 `rosgraph lint` — Static Analysis

Pre-launch verification of the ROS graph.

```
┌────────────────────────────────────┐
│           Loaders                  │
│  ┌───────────┐ ┌───────────────┐  │
│  │interface.  │ │ launch files  │  │
│  │yaml parser │ │ (clingwrap/   │  │
│  │            │ │  native)      │  │
│  └─────┬─────┘ └──────┬────────┘  │
│        └───────┬───────┘           │
│                ▼                   │
│  ┌──────────────────────────┐     │
│  │     Graph Model          │     │
│  └────────────┬─────────────┘     │
│               ▼                   │
│  ┌──────────────────────────┐     │
│  │   Analyzer DAG           │     │
│  │   (parallel execution)   │     │
│  │                          │     │
│  │  [topic_resolver]        │     │
│  │       ↓                  │     │
│  │  [type_mismatch_checker] │     │
│  │  [qos_compat_checker]    │     │
│  │  [naming_convention]     │     │
│  │  [disconnected_subgraph] │     │
│  │  [unused_node]           │     │
│  │  [launch_linter]         │     │
│  │  ...                     │     │
│  └────────────┬─────────────┘     │
│               ▼                   │
│  ┌──────────────────────────┐     │
│  │  Post-Processing         │     │
│  │  - suppression filter    │     │
│  │  - severity assignment   │     │
│  │  - deduplication         │     │
│  │  - differential (new     │     │
│  │    issues only for CI)   │     │
│  └────────────┬─────────────┘     │
│               ▼                   │
│  ┌──────────────────────────┐     │
│  │  Output Formatters       │     │
│  │  text, JSON, SARIF,      │     │
│  │  GitHub, JUnit           │     │
│  └──────────────────────────┘     │
└────────────────────────────────────┘
```

**Analyzer definition pattern (from Go analysis framework):**

```python
# Each analyzer is a value, not a subclass
topic_resolver = GraphAnalyzer(
    name="topic_resolver",
    doc="Resolves topic names to their message types across the graph",
    requires=[],
    result_type=TopicTypeMap,
    run=resolve_topics,
)

type_mismatch = GraphAnalyzer(
    name="type_mismatch",
    doc="Checks that all pub/sub on a topic agree on message type",
    requires=[topic_resolver],
    result_type=None,
    run=check_type_mismatches,
)
```

See [§7 Lint Rule Codes](#7-lint-rule-codes) for the full rule system.

**Launch file loading strategy:**

Three loader paths, not mutually exclusive, phased by tractability:

| Loader | Launch format | Extraction method | Phase | Limitations |
|---|---|---|---|---|
| YAML launch | YAML launch files | Direct parse | 1 | Limited expressiveness |
| `system.yaml` | Layer 2 schema | Direct parse | 1 | Requires manual authoring |
| Python launch AST | Standard `launch_ros` | AST pattern matching | 2 | Cannot handle dynamic logic (conditionals, loops) |

- **YAML launch files** are statically parseable — `rosgraph lint` can
  extract node declarations, remappings, and parameter overrides
  directly.
- **Python launch files** are imperative and Turing-complete, but most
  are declarative-in-spirit. AST-level pattern matching for common
  patterns (`Node()`, `LaunchConfiguration()`,
  `DeclareLaunchArgument()`) captures ~80% of real launch files without
  execution.
- **Layer 2 `system.yaml`** (§3.2) sidesteps the problem entirely —
  a static YAML file declaring the intended system composition. Launch
  files still run the system, but `system.yaml` is the lint/monitor
  source of truth for graph analysis.

The lint diagram's "launch files" loader encompasses all three paths.

### 3.6 `rosgraph monitor` — Runtime Reconciliation

Kubernetes-style reconciliation loop comparing declared vs. observed
graph state.

```
┌─────────────────────────────────────────────────┐
│              rosgraph monitor                    │
│                                                  │
│  ┌───────────────┐     ┌──────────────────────┐ │
│  │ Declared State │     │   Observed State     │ │
│  │ (from YAML /  │     │   (from DDS          │ │
│  │  interface    │     │    discovery)         │ │
│  │  files)       │     │                      │ │
│  └───────┬───────┘     └──────────┬───────────┘ │
│          │                        │              │
│          └──────────┬─────────────┘              │
│                     ▼                            │
│  ┌──────────────────────────────────┐            │
│  │     Reconciliation Engine        │            │
│  │                                  │            │
│  │  Level-triggered (not edge)      │            │
│  │  Idempotent                      │            │
│  │  Requeue with backoff            │            │
│  └──────────────┬───────────────────┘            │
│                 ▼                                │
│  ┌──────────────────────────────────┐            │
│  │     Diff Computation             │            │
│  │                                  │            │
│  │  - Missing/extra nodes           │            │
│  │  - Missing/extra topics          │            │
│  │  - QoS mismatches                │            │
│  │  - Type mismatches               │            │
│  │  - Parameter drift               │            │
│  └──────────────┬───────────────────┘            │
│                 ▼                                │
│  ┌──────────────────────────────────┐            │
│  │     Exporters                    │            │
│  │                                  │            │
│  │  - ROS topics (graph_diff msg)   │            │
│  │  - Prometheus /metrics endpoint  │            │
│  │  - Structured log output         │            │
│  │  - Alerting (via diagnostics)    │            │
│  └──────────────────────────────────┘            │
└─────────────────────────────────────────────────┘
```

**Reconciliation loop:**

```python
while running:
    declared = load_declared_graph(interface_files, launch_files)
    observed = scrape_live_graph(dds_discovery)

    diff = compute_graph_diff(declared, observed)

    if diff.has_issues():
        publish_diff(diff)           # ROS topic: /rosgraph/diff
        update_metrics(diff)         # Prometheus: rosgraph_missing_nodes, etc.
        emit_diagnostics(diff)       # /diagnostics for standard tooling

    publish_status(observed)         # ROS topic: /rosgraph/status

    # Adaptive interval: faster when drifting, slower when stable
    if diff.has_critical():
        sleep(1s)
    else:
        sleep(5s)
```

See [§8 Monitor Alert Rules](#8-monitor-alert-rules) for the alert
system.

**Relationship to graph-monitor:** `rosgraph monitor` is a new
implementation, not an extension of the existing graph-monitor package.
graph-monitor's value is its `rmw_stats_shim` and
`rosgraph_monitor_msgs` message definitions — these are reusable
regardless of architecture. However, graph-monitor lacks the
reconciliation engine (declared vs. observed diff) that is the core of
`rosgraph monitor`, and retrofitting it would constrain the design.

The integration path: adopt or align with graph-monitor's message
definitions (`rosgraph_monitor_msgs`), reimplement the graph scraping
and reconciliation, and offer to upstream the reconciliation capability
back to graph-monitor if its maintainers are interested.

### 3.7 `rosgraph test` — Contract Testing

Schema-driven verification of running nodes against their declarations.

Three testing modes (modelled on Schemathesis, Dredd, and Pact):

**Interface conformance** (Dredd model): Run a node, then
systematically verify its actual interface matches its
`interface.yaml`. Check every declared publisher is active, call every
declared service, verify every parameter exists with the declared type
and default.

**Fuzz testing** (Schemathesis model): Auto-generate messages matching
declared subscriber types, publish them, verify the node produces
outputs on declared publisher topics with correct types.

**Cross-node contract testing** (Pact model): Node A's
`interface.yaml` declares it subscribes to `/cmd_vel` (Twist). Node
B's `interface.yaml` declares it publishes `/cmd_vel` (Twist). The
contract test verifies they agree on type and QoS compatibility.

### 3.8 `rosgraph docs` — Documentation Generation

Auto-generated "Swagger UI for ROS nodes" — browsable API reference
docs from `interface.yaml`. Covers topics, services, actions,
parameters, QoS settings, and message type definitions.

Output formats: Markdown (for GitHub Pages / docs.ros.org), HTML
(standalone), JSON (for embedding in other tools).

### 3.9 `rosgraph breaking` — Breaking Change Detection

Compares two versions of `interface.yaml` and classifies changes:

| Classification | Examples |
|---|---|
| **Breaking** | Removed topic, changed message type, removed parameter, incompatible QoS change |
| **Dangerous** | Changed QoS (may affect connectivity), narrowed parameter range |
| **Safe** | Added optional parameter, added new publisher, widened parameter range |

Modelled on `buf breaking` and `graphql-inspector`.

### 3.10 `rosgraph discover` — Runtime-to-Spec Generation

Introspects a running node via DDS discovery and generates an
`interface.yaml` from the observed interface. The "slice of cake"
brownfield adoption path.

```bash
# Generate interface.yaml from a running node
rosgraph discover /lidar_processor -o nodes/lidar_processor/interface.yaml
```

Modelled on Terraform's `import` command.

### 3.11 Configuration

**`rosgraph.toml`** — single configuration file for all subcommands:

```toml
[lint]
select = ["TOP", "SRV", "QOS", "GRF"]  # enable these rule families
ignore = ["NME001"]                      # except this specific rule

[lint.per-package-ignores]
"generated_*" = ["ALL"]                  # skip generated packages
"*_test" = ["GRF002"]                    # allow unused nodes in tests

[generate]
plugins = ["cpp", "python"]
out_dir = "generated"

[output]
format = "text"                          # text | json | sarif | github

[ci]
new-only = true                          # only new issues (differential)
base-branch = "main"
```

### 3.12 Multi-Workspace Analysis

ROS 2 workspaces overlay each other (e.g., `ros_base` underlay + your
packages + a vendor overlay). When `rosgraph lint` analyzes your
workspace, it needs interface information from packages in the underlay.

The solution follows the Go analysis framework's per-package fact
caching pattern: installed `interface.yaml` files in
`share/<package>/interfaces/` (placed there by `rosgraph_auto_package()`
at install time) serve as cached analysis artifacts. `rosgraph lint`
reads these from the underlay without re-analyzing underlay packages,
only analyzing packages in the current workspace.

This is a Phase 2 concern. Phase 1 assumes a single workspace.

### 3.13 AI & Tooling Integration

`interface.yaml` and the `InterfaceDescriptor` IR (§3.3) are
machine-readable contracts describing a node's complete API. This
makes them natural integration points for AI-assisted development
tools and IDE infrastructure.

**AI as IR consumer.** The JSON-serialized `InterfaceDescriptor`
contains everything an LLM needs to understand a node's interface:
topics, types, QoS, parameters, lifecycle state. An AI agent can read
this to generate implementation code, write tests, suggest fixes, or
answer questions about the system — without parsing source code.

**MCP server.** A Model Context Protocol server exposing graph state,
lint results, and interface schemas enables AI coding tools (Claude
Code, Cursor, Copilot) to query the ROS graph as structured context.
"What topics does the perception pipeline publish?" answered from
the graph model, not from grep.

**AI-assisted discovery.** `rosgraph discover` (§3.10) generates
`interface.yaml` from a running system. The raw output from DDS
discovery is complete but lacks descriptions, rationale, and grouping.
An LLM can refine the generated spec — inferring descriptions from
topic names and message types, suggesting QoS profiles based on
message patterns, and grouping related interfaces.

**Language Server Protocol (LSP).** An LSP server for `interface.yaml`
enables IDE features beyond JSON Schema validation: hover for message
type definitions, go-to-definition for `$ref` targets, inline
diagnostics from `rosgraph lint`, and cross-file rename support. This
benefits both human developers and AI agents operating within IDE
contexts.

**Natural language to spec.** The constrained schema makes
`interface.yaml` a tractable generation target for LLMs. "I need a
node that subscribes to a lidar point cloud, filters it, and publishes
the result" produces a valid `interface.yaml` that `rosgraph generate`
can immediately scaffold into working code.

These are not Phase 1 deliverables, but the architecture should not
preclude them. The IR-based plugin protocol (§3.3) and structured
output formats (JSON, SARIF) are the key enablers — they exist for
code generation and CI, but AI consumers are a natural extension.

### 3.14 Scale & Fleet Considerations

§3.12 covers multi-workspace analysis. This section addresses
concerns beyond a single developer's workstation.

**Interface ownership.** In multi-team organizations, `interface.yaml`
files are shared contracts. The owner is typically the node author
(they define the interface), but downstream consumers depend on it.
Changes require coordination. rosgraph supports this via:
- `rosgraph breaking` (§3.9) — automated detection of breaking
  changes in CI, blocking merges that break downstream consumers.
- Installed interfaces in `share/<package>/interfaces/` — downstream
  teams depend on published interfaces without pulling source code.
- Semantic versioning alignment — the breaking/dangerous/safe
  classification maps to semver: breaking = major, dangerous = minor
  (review required), safe = patch.

**Multi-robot systems.** The `system.yaml` (Layer 2, §3.2) supports
namespaced node instances (`namespace: /robot1`). For multi-robot
systems, each robot's graph is a namespaced instance of the same
`system.yaml`. Fleet-level analysis — "which robots are running
interface version X?" — is out of scope for Phase 1–2 but the
architecture supports it: `rosgraph monitor` on each robot publishes
graph snapshots that a fleet-level aggregator can collect.

**Fleet monitoring.** `rosgraph monitor` (§3.6) runs per-robot. For
fleet-scale observability, the monitor's Prometheus exporter (M7)
enables standard fleet dashboards via Grafana. The `/rosgraph/diff`
topic on each robot can be bridged to a central system for aggregated
drift analysis. The architecture deliberately uses standard
observability patterns (Prometheus metrics, structured logs,
diagnostics topics) rather than inventing fleet-specific
infrastructure.

**Performance targets.** Build-time targets are stated in DP7 (100
packages in 5 seconds). Runtime targets for `rosgraph monitor`:
- Reconciliation cycle: < 500ms for a 200-node system
- Memory overhead: < 50MB resident for graph state
- CPU: < 5% of one core at steady-state (5s scrape interval)

These are design targets, not commitments — they guide architectural
decisions (e.g., choosing Rust for the diff engine).

### 3.15 colcon Integration

`colcon` uses a `VerbExtensionPoint` plugin system — any Python package
can register new verbs via `setup.cfg` entry points. Existing examples:
`colcon-clean` adds `colcon clean`, `colcon-cache` adds `colcon cache`.

The architecture is **`rosgraph` as standalone tool, `colcon-rosgraph`
as thin workspace wrapper**:

```
colcon-rosgraph (Python, verb plugin)
  └── delegates to → rosgraph (standalone binary)
```

This mirrors how `colcon-cmake` shells out to `cmake` — the colcon verb
handles workspace iteration, package ordering, and parallel execution;
the core tool handles single-package analysis.

**What maps naturally to colcon verbs:**

| Command | colcon verb | Notes |
|---|---|---|
| `rosgraph generate` | — | Already runs via `rosgraph_auto_package()` in `colcon build` |
| `rosgraph test` | — | Already runs via CTest in `colcon test` |
| `rosgraph lint` | `colcon lint` | Iterates packages in dependency order, parallel per-package lint |
| `rosgraph docs` | `colcon docs` | Generates docs per package, aggregates into workspace docs |
| `rosgraph discover` | `colcon discover` | Generates `interface.yaml` for all running nodes |
| `rosgraph breaking` | `colcon breaking` | Checks all packages against their previous interface versions |

**What doesn't fit:**

`rosgraph monitor` is a long-running daemon, not a build-and-exit verb.
It stays as a standalone command (or a `ros2 launch` node).

**Why both CLIs:**

- `colcon lint` for the workspace workflow — lint all packages, respect
  dependency order, parallel execution, workspace-level reporting.
- `rosgraph lint path/to/interface.yaml` for single-file use, CI
  pipelines, and environments without colcon.

**Language independence.** The colcon plugin is always Python (colcon
requires it), but it delegates to `rosgraph` via subprocess — so the
core tool's language is unconstrained. Rust, Python, or hybrid all work
identically. The colcon integration does not factor into the language
choice (§5).

The colcon plugin is a Phase 2 deliverable — Phase 1 focuses on the
standalone `rosgraph` tool. The plugin is trivial once the core tool
exists.

---

## 4. Phasing

### Phase 1 — Foundation

Deliver the core schema, basic code generation, and highest-value
static + runtime checks.

**Schema & generate:**
- G1-G10 (existing cake features — stabilize and adopt)
- G11 (lifecycle nodes — blocks nav2/ros2_control adoption)
- G14 (schema versioning — needed before v1.0)

**Lint (P0 rules):**
- L1 (topic type mismatch), L2 (QoS compatibility), L3 (disconnected
  subgraph)
- L5 (SARIF output), L6 (differential analysis)

**Monitor (P0 features):**
- M1 (declared-vs-observed diff), M2 (missing node alerting),
  M5 (graph snapshots)

### Phase 2 — Adoption Enablers

Lower barriers for existing codebases. Fill out the rule set.

**Schema & generate:**
- G12 (timers), G13 (nested parameters), G15 (mixins)
- O1 (`rosgraph docs`), O2 (`rosgraph discover`)

**Lint (P1 rules + infrastructure):**
- L4 (launch validation), L7 (naming), L8 (unused node),
  L9 (parameter validation), L10 (circular deps)
- L11 (inline suppression), L12 (per-package config),
  L13 (`--add-noqa`), L14 (semantic validation)

**Monitor (P1 features):**
- M3 (QoS drift), M4 (runtime type mismatch), M6 (topic stats),
  M8 (unexpected node), M9 (health diagnostics)

### Phase 3 — Scale the Toolchain

Enable community extension and advanced analysis.

**Schema & generate:**
- G16 (plugin architecture), G17 (callback groups),
  G19 (system composition schema)
- O3 (`rosgraph breaking`), O4 (`rosgraph test`)

**Lint:**
- L15 (interface coverage)

**Monitor:**
- M7 (Prometheus endpoint), M10 (adaptive scrape),
  M11 (lifecycle state)

### Phase 4 — Ecosystem Integration

Future-proofing and niche use cases.

- G18 (middleware bindings)
- O5 (`rosgraph policy` — SROS 2 security policies)
- M12 (runtime interface coverage)

### Adoption Path

rosgraph is unlikely to reach `ros_core` initially — that requires
broad consensus and a high stability bar. A more realistic progression:

1. **`ros-tooling` organization** (where graph-monitor already lives) —
   institutional backing, CI infrastructure, release process.
2. **REP (ROS Enhancement Proposal)** for the `interface.yaml` schema —
   formalizes the declaration format as a community standard.
3. **docs.ros.org tutorial integration** — if the "write your first
   node" tutorial uses `interface.yaml`, every new ROS developer learns
   it from day one. This is the highest-leverage adoption path.
4. **`ros_core` proposal** — after demonstrated adoption across multiple
   distros, propose for inclusion in a future distribution.

---

## 5. Language Choice

The implementation language is an open decision for the WG. The
trade-offs are structural, not preferential.

### Option A: Rust

Follows Ruff's model. Speed as an architectural property.

| Axis | Assessment |
|---|---|
| Performance | Best. Single-pass analysis, zero-cost abstractions, no GC pauses. Achieves the "100 packages in 5s" target. |
| Contribution barrier | Highest. Most ROS contributors know C++/Python, not Rust. |
| Ecosystem fit | Moderate. `rclrs` exists but is not tier-1. CLI tools don't need ROS client library integration. |
| Deployment | Single static binary. No runtime dependencies. |
| Plugin story | WASM plugins (Extism) or process-based (protoc model). |

### Option B: Python

Follows the ROS 2 ecosystem convention.

| Axis | Assessment |
|---|---|
| Performance | Weakest. 10-100x slower than Rust for analysis workloads. May not meet performance targets. |
| Contribution barrier | Lowest. Every ROS developer knows Python. |
| Ecosystem fit | Best. cake is Python. `launch_ros` is Python. Direct reuse of existing parsing libraries. |
| Deployment | Requires Python runtime. `pip install` or ROS package. |
| Plugin story | Native Python plugins. Trivial to write. |

### Option C: Rust core + Python bindings

Hybrid via PyO3. Performance-critical core (parsing, graph model, diff
engine, lint rules) in Rust; Python CLI and plugin layer on top.

| Axis | Assessment |
|---|---|
| Performance | Near-Rust for analysis; Python overhead for CLI/plugin dispatch only. |
| Contribution barrier | Moderate. Core contributors need Rust; plugin authors use Python. |
| Ecosystem fit | Good. Python-facing API integrates with ROS ecosystem. |
| Deployment | Python package with native extension. Requires build toolchain for distribution. |
| Plugin story | Python plugins (native) + WASM plugins (for sandboxing). |

### Decision factors

The choice depends on which constraint the WG prioritizes:
- If **speed** is the binding constraint → Rust or hybrid
- If **community contribution** is the binding constraint → Python
- If **both matter** → hybrid, accepting the build complexity

Note: the colcon integration (§3.15) does not constrain this choice.
The `colcon-rosgraph` plugin is always Python but delegates to the
`rosgraph` binary via subprocess, so the core tool can be any language.

---

## 6. Feature List

### Schema & Code Generation (`rosgraph generate`)

| # | Feature | Priority | Description |
|---|---------|----------|-------------|
| G1 | YAML interface declaration | P0 | Single `interface.yaml` per node declaring all ROS 2 entities |
| G2 | JSON Schema validation | P0 | Structural validation with IDE autocompletion via YAML Language Server |
| G3 | C++ code generation | P0 | Typed context, pub/sub/srv/action wrappers, component registration |
| G4 | Python code generation | P0 | Dataclass context, pub/sub/srv/action wrappers |
| G5 | Parameter generation | P0 | Delegates to `generate_parameter_library` (backward-compatible) |
| G6 | QoS declaration | P0 | Required for pub/sub, supports all DDS QoS policies |
| G7 | Parameterized QoS | P0 | `${param:name}` references in QoS fields |
| G8 | Dynamic topic names | P0 | `${param:name}` and `${for_each_param:name}` |
| G9 | Composition pattern | P0 | Has-a `Node`, not is-a `Node` |
| G10 | Zero-boilerplate build | P0 | `rosgraph_auto_package()` CMake macro |
| G11 | Lifecycle node support | P0 | `lifecycle: managed` in node spec |
| G12 | Timer declarations | P1 | `timers:` section with period, callback name |
| G13 | Nested parameters | P1 | Hierarchical parameter structures (parity with gen_param_lib) |
| G14 | Schema versioning | P1 | `schema_version` field with migration tooling |
| G15 | Mixins / shared fragments | P1 | `$ref` to common interface fragments |
| G16 | Plugin architecture | P2 | IR-based pipeline, standalone plugins per language |
| G17 | Callback group declarations | P2 | `callback_groups:` with entity assignment |
| G18 | Middleware bindings | P3 | Protocol-specific config (DDS, Zenoh) |
| G19 | System composition schema | P2 | Multi-node graph declaration (`system.yaml`, Layer 2) |

### Static Analysis (`rosgraph lint`)

| # | Feature | Priority | Description |
|---|---------|----------|-------------|
| L1 | Topic type mismatch detection | P0 | Flag when pub and sub on same topic disagree on message type |
| L2 | QoS compatibility checking | P0 | Flag incompatible QoS profiles (reliability, durability, deadline) |
| L3 | Disconnected subgraph detection | P0 | Flag nodes/topics with no connections |
| L4 | Launch file validation | P0 | Detect undefined node refs, invalid remaps, unresolved substitutions |
| L5 | SARIF / CI output | P0 | Structured output for GitHub Security tab, PR annotations |
| L6 | Differential analysis | P0 | `--new-only` reports only issues introduced since base branch |
| L7 | Naming convention enforcement | P1 | Check names against configurable patterns |
| L8 | Unused node detection | P1 | Flag nodes declared but not in any launch config |
| L9 | Parameter validation | P1 | Check values against declared types, ranges, validators |
| L10 | Circular dependency detection | P1 | Flag service/action chains that could deadlock |
| L11 | Inline suppression | P1 | `# rosgraph: noqa: TOP001` in launch/YAML files |
| L12 | Per-package configuration | P1 | Override rules per package via `rosgraph.toml` |
| L13 | `--add-noqa` for adoption | P1 | Generate suppression comments for all existing issues |
| L14 | Semantic validation | P1 | Full type resolution, QoS compatibility checks |
| L15 | Interface coverage reporting | P2 | Which declared topics/services are exercised in tests |

### Runtime Monitoring (`rosgraph monitor`)

| # | Feature | Priority | Description |
|---|---------|----------|-------------|
| M1 | Declared-vs-observed graph diff | P0 | Compare declared interfaces against live DDS discovery |
| M2 | Missing node alerting | P0 | Alert when a declared node is not present |
| M3 | QoS drift detection | P0 | Alert when observed QoS differs from declared |
| M4 | Type mismatch detection (runtime) | P0 | Alert when observed types differ from declaration |
| M5 | Graph snapshot publishing | P0 | Periodic `rosgraph_monitor_msgs/Graph` snapshots |
| M6 | Topic statistics | P1 | Message rate, latency, queue depth per topic |
| M7 | Prometheus /metrics endpoint | P1 | Export graph metrics for Grafana dashboards |
| M8 | Unexpected node detection | P1 | Alert on nodes present but not declared |
| M9 | Health diagnostics integration | P1 | Publish to `/diagnostics` for standard ROS tooling |
| M10 | Adaptive scrape interval | P2 | Faster scraping when drift detected, slower when stable |
| M11 | Lifecycle state monitoring | P2 | Track lifecycle transitions against expectations |
| M12 | Interface coverage tracking | P2 | Which declared interfaces are exercised at runtime |

### Other Subcommands

| # | Feature | Subcommand | Priority | Description |
|---|---------|------------|----------|-------------|
| O1 | Documentation generation | `rosgraph docs` | P1 | Auto-generated API reference from schema |
| O2 | Runtime-to-spec discovery | `rosgraph discover` | P1 | Introspect running nodes → `interface.yaml` |
| O3 | Breaking change detection | `rosgraph breaking` | P2 | Detect breaking interface changes across releases |
| O4 | Contract testing | `rosgraph test` | P2 | Schema-driven verification of running nodes |
| O5 | Security policy generation | `rosgraph policy` | P3 | Auto-generate SROS 2 policies from schema |

---

## 7. Lint Rule Codes

Rule codes use hierarchical prefix system (modelled on Ruff). Rules
can be selected at any granularity: `TOP` (all topic rules),
`TOP001` (specific rule).

| Prefix | Category | Example rules |
|--------|----------|---------------|
| `TOP` | Topic rules | `TOP001` type mismatch, `TOP002` no subscribers, `TOP003` naming convention |
| `SRV` | Service rules | `SRV001` unmatched client, `SRV002` type mismatch |
| `ACT` | Action rules | `ACT001` unmatched client, `ACT002` type mismatch |
| `PRM` | Parameter rules | `PRM001` missing default, `PRM002` type violation, `PRM003` undeclared |
| `QOS` | QoS rules | `QOS001` reliability mismatch, `QOS002` durability incompatible, `QOS003` deadline violation |
| `LCH` | Launch rules | `LCH001` undefined node ref, `LCH002` invalid remap, `LCH003` unresolved substitution |
| `GRF` | Graph-level rules | `GRF001` disconnected subgraph, `GRF002` unused node, `GRF003` circular dependency |
| `NME` | Naming rules | `NME001` topic naming convention, `NME002` node naming convention |
| `SAF` | Safety rules | `SAF001` insufficient redundancy, `SAF002` single point of failure, `SAF003` unmanaged safety node |
| `TF` | TF frame rules | `TF001` undeclared frame_id, `TF002` broken frame chain |

**Rule lifecycle:** preview → stable → deprecated → removed. New rules
always enter as preview.

**Fix applicability:** Safe (preserves semantics), unsafe (may alter
behaviour), display-only (suggestion). Per-rule override via config.

---

## 8. Monitor Alert Rules

| Alert | Condition | Grace period | Severity |
|---|---|---|---|
| `NodeMissing` | Declared node not observed | 10s | critical |
| `UnexpectedNode` | Observed node not declared | 30s | warning |
| `TopicMissing` | Declared topic not present | 5s | critical |
| `QoSMismatch` | Declared QoS ≠ observed QoS | 0s | error |
| `TypeMismatch` | Declared msg type ≠ observed | 0s | critical |
| `ThroughputDrop` | Rate < expected minimum | 30s | warning |

Grace periods prevent flapping during startup and transient states.
All thresholds (grace period, severity) are configurable via
`rosgraph.toml` — see §11.3 for safety-critical overrides.

---

## 9. Existing ROS 2 Ecosystem

### 9.1 Maturity Matrix

| Tool | Stars | Contributors | Last active | Maturity | Bus factor |
|---|---|---|---|---|---|
| **generate_parameter_library** | 353 | 41 | 2026-02 | Production | Healthy |
| **ros2_tracing** | 237 | 30 | 2026-02 | Production (QL1) | Healthy |
| **topic_tools** | 126 | 25 | 2025-08 | Mature | Healthy |
| **launch_ros** | 78 | 71 | 2026-02 | Core infrastructure | Healthy |
| **cake** | 36 | 1 | 2026-02 | Early-stage | 1 (risk) |
| **graph-monitor** | 31 | 3 | 2025-11 | Mid-stage | Low |
| **nodl** | 10 | 7 | 2022-11 | Dormant | N/A |
| **clingwrap** | 9 | 1 | 2026-02 | Early-stage | 1 (risk) |
| **breadcrumb** | 6 | 1 | 2026-02 | Early-stage | 1 (risk) |
| **HAROS** (ROS 1) | 197 | — | 2021-09 | Abandoned | N/A |
| **CARET** | 97 | 18 | active | Mature (Tier IV) | Healthy |

### 9.2 Tool Assessments

**cake** — Declarative code generation. `interface.yaml` → C++ and
Python node scaffolding. Functional pattern (has-a Node, not is-a
Node). The fundamental bet is correct: making the interface declaration
the source of truth for code generation is the only way to prevent
schema-code drift. Core design decisions (YAML-driven,
composition-based, schema-validated, codegen-first) are sound.
cake's author is a WG member; rosgraph's Layer 1 schema builds
directly on cake's format, and G1–G10 represent stabilizing cake's
capabilities under the rosgraph umbrella — addressing the bus-factor
risk while preserving the design. Gaps: no lifecycle support, no
timers, no nested parameters, no formal IR, no plugin architecture,
no runtime-to-spec generation.

**generate_parameter_library** — The most mature tool in the space.
Production-proven in MoveIt2 and ros2_control. Rich validation. The
unification path: the `parameters:` section of `interface.yaml` IS the
`generate_parameter_library` format (already demonstrated in cake).
rosgraph delegates to `generate_parameter_library` at build time rather
than reimplementing parameter generation. The key invariant: a
standalone gen_param_lib YAML file works as-is when placed in the
`parameters:` block of `interface.yaml`. Ownership transfer to
`ros-tooling` would be ideal but is not required — schema compatibility
is sufficient.

**graph-monitor** — Official ROSGraph WG backing. Publishes structured
graph messages. The `rmw_stats_shim` approach is architecturally sound.
Gap: can report *what exists* but not *what's wrong* — no comparison
against a declared spec.

**breadcrumb + clingwrap** — Proves the concept of static graph
extraction from launch files. The tight coupling to clingwrap's
non-standard launch API is the primary concern. Static analysis should
work with standard `launch_ros` patterns.

**nodl** — Dormant since 2022. Correct problem identification but
fatal flaw: no code generation. Superseded by cake's YAML approach.
Key lesson: **a description format without code generation is a
non-starter.**

**ros2_tracing + CARET** — The most mature dynamic analysis tools.
QL1 certification, production-proven at Tier IV. Complementary to
rosgraph: tracing provides instrumentation, CARET provides latency
analysis, rosgraph provides graph structure analysis.

### 9.3 Gap Analysis

| Category | Capability | Current tool | Status |
|---|---|---|---|
| **Schema** | Node interface declaration | cake / nodl / gen_param_lib | cake early; nodl dead; gpl params-only |
| **Codegen** | Static graph from launch files | breadcrumb + clingwrap | Early-stage, solo dev |
| **Runtime** | Runtime graph monitoring | graph-monitor | Mid-stage, institutional |
| **Runtime** | Runtime tracing | ros2_tracing | Mature, production |
| **Runtime** | Latency analysis | CARET | Mature, Tier IV |
| **Runtime** | Graph visualisation | Foxglove, Dear RosNodeViewer | Mature but live-only |
| **Runtime** | **Graph diff (expected vs. actual)** | **Nothing** | **Major gap** |
| **Static** | **Graph linting (pre-launch)** | **Nothing** | **Major gap** |
| **Static** | **QoS static analysis** | breadcrumb (partial) | Early-stage |
| **Static** | **CI graph validation** | **Nothing** | **Major gap** |
| **Docs** | **Node API documentation** | **Nothing** (hand-written only) | **Major gap** |
| — | **Behavioural properties** | **Nothing** (HPL was ROS 1) | **Major gap** |

---

## 10. Prior Art

Organized by what we borrow, not by framework. Each framework appears
once at its primary contribution.

### 10.1 Schema Design

#### AsyncAPI

The closest structural match to ROS topics. Version 3 cleanly separates
channels, messages, operations, and components at the top level.

**What to borrow:**
- **Structural separation.** `publishers`, `subscribers`, `services`,
  `actions`, `parameters` as peer top-level sections.
- **`components` + `$ref` pattern.** Define QoS profiles or common
  parameter sets once, reference everywhere.
- **Trait system.** Define a `reliable_sensor` trait with QoS settings,
  apply to multiple publishers. Traits merge via JSON Merge Patch
  (RFC 7386).
- **Protocol bindings.** Core schema stays middleware-agnostic;
  DDS-specific QoS, Zenoh settings, or shared-memory config in a
  `bindings:` block.
- **Parameterized addresses.** Topic name templates
  (`sensors/{robot_name}/lidar`) map to ROS 2 namespace/remapping and
  `${param:name}` syntax.

**Gaps:** No services (as typed req/res pair), no actions, no
parameters, no lifecycle, no timers, no TF frames. Single-application
scope (which is actually the right scope for a node interface).

#### Smithy (AWS)

Protocol-agnostic interface definition language. Shapes decorated with
traits.

**What to borrow:**
- **Typed, composable traits** for extensible metadata — the most
  powerful metadata mechanism surveyed:
  ```
  @qos(reliability: "reliable", depth: 10)
  @lifecycle(managed: true)
  @parameter_range(min: 0.0, max: 10.0)
  @frame_id("base_link")
  ```
- **Mixins** for shared structure. A `lifecycle_diagnostics` mixin adds
  a diagnostics publisher and period parameter to any node that
  includes it.
- **Resource lifecycle operations** — maps to ROS 2 lifecycle node
  transitions.

#### CUE

Constraint-based configuration language where types and values are the
same thing. Not a codegen tool — a validation tool.

**What to borrow:**
- **Constraints as types.** `voxel_size: float & >=0.01 & <=1.0`. The
  JSON Schema equivalent (`minimum`, `maximum`, `enum`) is already
  used by the existing `interface.schema.yaml`.
- **Incremental constraints.** Base schema + deployment-specific
  overlays (e.g., production QoS profiles layered onto a base
  `interface.yaml`).
- **Configuration validation.** Validate that launch parameter
  overrides are compatible with a node's declared interface.

### 10.2 Pipeline & Code Generation

#### Protocol Buffers / Buf CLI

The single most important architectural lesson: **an intermediate
representation (IR) between parsing and generation**.

```
interface.yaml ──> [Parser/Validator] ──> InterfaceDescriptor (IR)
                                            ├──> [Plugin: C++]    ──> scaffolding
                                            ├──> [Plugin: Python] ──> scaffolding
                                            ├──> [Plugin: Docs]   ──> API reference
                                            └──> [Plugin: Launch] ──> templates
```

**What to borrow:**
- **IR-based plugin protocol.** Standalone executables consuming a
  serialized `InterfaceDescriptor` via stdin/file. Community members
  write `rosgraph-gen-rust` without touching the core codebase.
- **Config-driven generation** (`buf.gen.yaml` pattern):
  ```yaml
  version: 1
  plugins:
    - name: cpp
      out: generated/cpp
      options: { lifecycle: managed }
    - name: python
      out: generated/python
  ```
- **Validation as separate layers.** Structural (does the YAML parse?)
  → semantic (do referenced types exist?) → breaking (did the interface
  change incompatibly?). Maps to `rosgraph lint`, `rosgraph validate`,
  `rosgraph breaking`.
- **Deterministic, reproducible output.** Same inputs → byte-identical
  output. CI can verify generated code is up to date.

**What to borrow from Buf CLI specifically:**
- `buf lint` — configurable schema linting with ~50 rules by category.
  Config-driven rule selection.
- `buf breaking` — breaking change detection between schema versions.
- Integrated toolchain: `buf generate`, `buf lint`, `buf breaking`,
  `buf format` as subcommands of one tool.

#### TypeSpec (Microsoft)

**What to borrow:**
- **Multi-emitter architecture.** One spec, many outputs:
  ```
  interface.yaml ──> C++ emitter       ──> node_interface.hpp
                 ──> Python emitter    ──> interface.py
                 ──> Docs emitter      ──> node_api_reference.md
                 ──> Launch emitter    ──> default_launch.py
                 ──> Graph emitter     ──> rosgraph_monitor_msgs/NodeInterface
  ```
- **Emitter-specific validation.** Each emitter adds its own checks
  (e.g., C++ emitter warns about names that produce invalid C++
  identifiers).

#### OpenAPI

**What to borrow:**
- **The "Swagger UI" experience.** Auto-generated interactive
  documentation from a schema. A "Swagger UI for ROS nodes" where every
  node has browsable API docs showing topics, services, actions,
  parameters, QoS, and message type definitions — generated from
  `interface.yaml`.
- **JSON Schema integration.** OpenAPI 3.1 aligned fully with JSON
  Schema. The existing `interface.schema.yaml` (JSON Schema Draft
  2020-12) is the right foundation.

### 10.3 Static Analysis Architecture

#### Ruff

A Python linter written in Rust. Relevant not for Python linting but as
the **best-in-class architecture for building a rule-based analysis
tool**.

**What to borrow:**

| Ruff pattern | rosgraph equivalent |
|---|---|
| Rule enum + compile-time registry | `Rule` enum: `TOP001`, `SRV001`, `QOS001`, `GRF001` |
| Hierarchical prefix codes | `TOP` (topic), `SRV` (service), `ACT` (action), `QOS`, `GRF` (graph) |
| Single-pass traversal | Build graph model once, run all rules in one walk |
| Safe/unsafe fix classification | Safe: add missing QoS. Unsafe: rename topic. Display-only: suggest restructure |
| Preview → stable lifecycle | Same graduation for new rules |
| Per-file-ignores | Per-package-ignores, per-launch-file-ignores |
| Inline suppression | `# rosgraph: noqa: TOP001` |
| SARIF output | GitHub Security tab integration |
| Monolithic, no plugins initially | All rules built-in. WASM plugins later |
| Zero-config defaults | Small, high-confidence default rule set |
| `--add-noqa` for gradual adoption | Essential for existing ROS workspaces |

**Key architectural lesson:** Speed is an architectural property, not an
optimisation. Rust + hand-written parser + single-pass + parallel
package processing + content caching + compile-time codegen.

#### Go Analysis Framework

The gold standard for pluggable static analysis architecture. Used by
`go vet`, gopls, and golangci-lint.

**What to borrow:**

```
GraphAnalyzer {
    name:        str
    doc:         str
    requires:    [GraphAnalyzer]      # horizontal deps
    result_type: Type | None          # typed output for dependent analyzers
    fact_types:  [Fact]               # cross-package facts
    run:         (GraphPass) → (result, [Diagnostic])
}

GraphPass {
    graph:       ComputationGraph     # the full graph model
    node:        NodeInterface        # current node under analysis
    types:       MessageTypeDB        # all known msg/srv/action types
    qos:         QoSProfileDB         # QoS profiles in the graph
    result_of:   {Analyzer: Any}      # results from required analyzers
    report:      (Diagnostic) → void
    import_fact: (scope, Fact) → bool
    export_fact: (scope, Fact) → void
}
```

Key patterns:
1. **Analyzers as values, not subclasses** — trivially composable
2. **Pass as abstraction barrier** — same analyzer in CLI, IDE, CI
3. **Horizontal dependencies** via `Requires`/`ResultOf` — typed data
   flow between analyzers
4. **Vertical facts** for cross-package analysis — cached per-package
   results enabling separate modular analysis
5. **Action graph** — 2D grid (analyzer x package), independent actions
   execute in parallel

#### golangci-lint

**What to borrow:**
- **Meta-linter pattern.** One CLI, one config, one output format
  wrapping many analyzers.
- **Shared parse.** All analyzers share one AST/model parse.
- **Post-processing pipeline.** `noqa` filter → exclusion rules →
  severity assignment → deduplication → output formatting.
- **Differential analysis.** `new-from-merge-base: main` reports only
  issues in code changed since the base branch. Critical for CI
  adoption in large codebases.

#### Spectral

**What to borrow:**
- **YAML-native lint rules** that work directly on `interface.yaml`
  without language-specific parsing. Custom rulesets in YAML — a
  robotics engineer can author a rule without knowing Rust or C++.
  Low barrier to writing new rules.

### 10.4 Runtime Monitoring Architecture

#### OpenTelemetry

Collector pipeline: Receiver → Processor → Exporter. Connectors join
pipelines and enable signal type conversion.

**What to borrow:**
- **Pipeline architecture** for `rosgraph monitor`.
- **Auto-instrumentation.** Two complementary paths:
  - *Runtime observation* (zero-code): DDS discovery provides the graph
    without modifying any node.
  - *Code-generated instrumentation*: rosgraph-generated code embeds
    topic stats, heartbeats, structured logging.
  - The **three-way comparison** (declared vs. runtime-observed vs.
    self-reported) catches issues that any two-way comparison misses.

#### Prometheus

**What to borrow:**
- **Pull model.** Periodic scraping produces consistent point-in-time
  snapshots. Absence of data is itself a signal (node is down).
- **Alerting rules** with `for` durations to prevent flapping.
- **Metric types mapping:**

  | Prometheus type | ROS topic statistics equivalent |
  |---|---|
  | Counter | Messages published (total), dropped messages |
  | Gauge | Active subscribers, queue depth, alive nodes |
  | Histogram | Inter-arrival times, message sizes, latency distribution |

#### Kubernetes Controllers

**What to borrow:**
- **Level-triggered reconciliation** (not edge-triggered). React to the
  *current difference* between desired and actual state, not to
  individual change events. If an event is missed, the next
  reconciliation still catches the drift.
- **Idempotent.** Running reconciliation twice with the same state
  produces the same diff and alerts.
- **Requeue with backoff.** After detecting drift, recheck sooner (1s).
  If drift persists, escalate.
- **Status reporting.** Maintained separately from the declared spec,
  enabling external tools to query current state independently.

### 10.5 Contract Testing & Verification

| Framework | What it does | What to borrow for `rosgraph test` |
|---|---|---|
| **Schemathesis** | Fuzz a live API against its OpenAPI spec. Auto-generates test cases from schema. | Fuzz a running node against `interface.yaml` — auto-generate messages matching declared types, verify outputs. |
| **Dredd** | Start a live server, send requests matching the spec, validate responses. The spec IS the test plan. | Run a node, systematically verify its interface matches declaration. Call every service, check every publisher. |
| **Pact** | Consumer-driven contract testing. Consumer declares expectations; provider verifies. | Cross-node contract verification: Node A subscribes to `/cmd_vel` (Twist), Node B publishes it. Verify they agree on type. |
| **gRPC health + reflection** | Standardized health checking + runtime introspection of services/methods. | Health reporting interface that rosgraph-generated nodes expose automatically. Runtime introspection vs. declared interface. |
| **graphql-inspector** | Schema diff (breaking/dangerous/safe). Coverage: which fields are actually queried. | Interface coverage: "which declared topics are exercised in tests?" Schema diff between interface versions. |

### 10.6 ROS Domain Prior Art: HAROS

The High-Assurance ROS framework (University of Minho, 2016–2021). The
only tool that accomplished Goals 3–4 for ROS, but only for ROS 1.

**Pipeline:** Package discovery → CMake parsing → launch file parsing →
source code parsing (libclang for C++, limited Python AST) →
computation graph assembly → plugin-based analysis → JSON export.

**The metamodel.** Formal classes for the ROS graph: `Node`,
`NodeInstance`, `Topic`, `Service`, `Parameter`, plus typed link classes
(`PublishLink`, `SubscribeLink`, etc.) carrying source conditions and
dependency sets. This metamodel is HAROS's most transferable
contribution.

**HPL (HAROS Property Language).** Behavioural properties for
message-passing systems:
```
globally: no /cmd_vel {linear.x > 1.0}           # speed limit
globally: /bumper causes /stop_cmd                 # response
globally: /cmd_vel requires /trajectory within 5s  # precedence
```

HPL drove three verification paths from a single spec: model checking
(Electrum/Alloy), runtime monitors (generated), and property-based
testing (Hypothesis strategies).

**Why it died for ROS 2.** The extraction pipeline assumes catkin,
`rospack`, XML launch files, `ros::NodeHandle`. ROS 2 changed
everything. The maintainer closed ROS 2 support as *wontfix*.

**What to borrow:** Metamodel, HPL's scope+pattern+event structure,
plugin separation (source-level vs. model-level), one spec → multiple
verification modes.

**What to do differently:** Use declarations (`interface.yaml`) as
primary source of truth (not source code parsing); support ROS 2
concepts HAROS never had (QoS, lifecycle, components, actions, DDS
discovery).

---

## 11. Safety & Certification

rosgraph is not a safety tool — it is a development and verification
tool that produces artifacts useful in safety cases. This section maps
rosgraph capabilities to the evidence types required by safety
standards.

### 11.1 Relevant Standards

| Standard | Domain | How rosgraph helps |
|---|---|---|
| **IEC 61508** | General functional safety | Design verification evidence (graph analysis), runtime monitoring |
| **ISO 26262** | Automotive | Interface specification (`interface.yaml` as design artifact), static verification |
| **IEC 62304** | Medical device software | Software architecture documentation, traceability |
| **DO-178C** | Aerospace | Requirements traceability, structural coverage analysis |
| **ISO 13482** | Service robots | Interface documentation, runtime monitoring |
| **ISO 21448 (SOTIF)** | Safety of intended functionality | Graph analysis for identifying missing/unexpected interfaces |

### 11.2 Artifact-to-Evidence Mapping

| rosgraph artifact | Evidence type | Useful for |
|---|---|---|
| `interface.yaml` | Software architecture description | Design phase documentation |
| `rosgraph lint` SARIF output | Static analysis results | Verification evidence |
| `rosgraph monitor` logs | Runtime verification evidence | Validation phase |
| `rosgraph test` results | Interface conformance evidence | Integration testing |
| `rosgraph breaking` output | Change impact analysis | Change management |
| `rosgraph docs` output | API documentation | Design review |

### 11.3 Configurable Safety Levels

Monitor alert grace periods (§8) and severity levels must be
configurable for safety-critical deployments:

```toml
[monitor.alerts]
NodeMissing = { grace_period_ms = 1000, severity = "critical" }   # 1s for surgical robot
UnexpectedNode = { grace_period_ms = 5000, severity = "error" }
TopicMissing = { grace_period_ms = 500, severity = "critical" }
```

The defaults in §8 are tuned for general robotics. Safety-critical
deployments override them via `rosgraph.toml`.

### 11.4 Behavioral Properties (Future)

Structural analysis (Phase 1–2) proves the graph is correctly wired —
a necessary precondition for behavioral safety. Behavioral analysis
(Phase 3+) proves temporal and causal properties:

```
globally: /emergency_stop causes /motor_disable within 100ms
globally: no /cmd_vel {linear.x > max_speed}
globally: /heartbeat absent_for 500ms causes /safe_stop
```

This capability, inspired by HAROS HPL (§10.6), is where the deeper
safety value lies. The structural graph model (§3.1) is designed to
be extensible to behavioral annotations without schema redesign.

### 11.5 Safety-Relevant Lint Rules (Future)

| Rule | Description | Phase |
|---|---|---|
| `SAF001` | Critical subscriber has < N publishers (no redundancy) | 2 |
| `SAF002` | Single point of failure in graph topology | 2 |
| `SAF003` | Safety-critical node is not lifecycle-managed | 2 |
| `TF001` | Declared `frame_id` not published by any node in graph | 2 |
| `TF002` | Frame chain broken (no transform path between declared frames) | 3 |

These rules are not in Phase 1 but the analyzer architecture (§3.5)
supports adding them without architectural changes.

---

## 12. Scope & Limitations

### When Not to Use rosgraph

rosgraph adds value when the cost of interface bugs exceeds the cost
of maintaining declarations. This trade-off favors rosgraph in
multi-node systems, team environments, and production deployments.
It does not favor rosgraph in every context:

- **Quick prototyping.** If you're experimenting with a single node
  and will throw it away next week, `interface.yaml` is overhead.
  Use standard `rclcpp` / `rclpy` directly.
- **Single-node packages.** A package with one node and no
  cross-package interfaces gets minimal lint value. The code
  generation may still be worthwhile for parameter validation.
- **Highly dynamic interfaces.** Nodes that create publishers and
  subscribers at runtime based on dynamic conditions (e.g., a
  plugin host that discovers its interface at startup) are outside
  scope (DP12). rosgraph can declare the static portion and flag
  the dynamic portion as unexpected, but it cannot generate code
  for interfaces it doesn't know about at build time.

### Known Limitations

**Spec-code drift for business logic.** Code generation covers the
structural skeleton (pub/sub creation, parameter declaration, lifecycle
transitions). Business logic is hand-written. If a developer adds an
undeclared publisher inside a callback, `rosgraph lint` won't catch it
at build time — only `rosgraph monitor` flags it at runtime as
`UnexpectedTopic`. This is a fundamental limitation of any
declaration-based approach: the declaration describes the intended
interface, not the implementation.

**Launch file coverage.** Python launch files are Turing-complete.
AST pattern matching (§3.5) handles common declarative patterns but
cannot resolve dynamic logic (conditionals based on environment
variables, loops generating node sets). `system.yaml` (Layer 2) is
the escape hatch for systems that need full static analyzability.

**Ecosystem bootstrapping.** rosgraph's cross-package analysis (type
mismatch detection, contract testing) requires multiple packages to
have `interface.yaml`. The single-package value proposition is code
generation and parameter validation. Cross-package value grows with
adoption. `rosgraph discover` (§3.10) lowers the barrier by generating
specs from running systems, but the generated specs require human
review and refinement.

**Scope of this proposal.** This document covers 51 features across
7 subcommands. Not all will be built. Phase 1 (§4) is the commitment
— the minimum viable tool that delivers value. Later phases are
contingent on adoption and contributor capacity.

---

## 13. Resolved Questions

The following questions were raised during the proposal drafting process
and have been resolved. Answers are integrated into the relevant
sections of this document.

| # | Question | Resolution | Section |
|---|----------|------------|---------|
| 1 | Dynamic interfaces | Out of scope — rosgraph covers declared interfaces only (Design Principle 12). Undeclared runtime interfaces are flagged as `UnexpectedTopic` by monitor. | §2 |
| 2 | Launch substitution evaluation | Three-path loader strategy: YAML launch (direct parse), `system.yaml` (static), Python launch AST (pattern matching). | §3.5 |
| 3 | Behavioural properties | Structural first (Phase 1–2), behavioural later (Phase 3+) if adoption warrants it (Design Principle 13). | §2 |
| 4 | `generate_parameter_library` unification | Keep as standalone, maintain schema compatibility. rosgraph delegates to gen_param_lib at build time. | §9.2 |
| 5 | Multi-workspace analysis | Per-package fact caching via installed `interface.yaml` files. Phase 2 concern. | §3.12 |
| 6 | Launch file extraction without clingwrap | Partial AST extraction for standard `launch_ros` patterns, with `system.yaml` as fully-static alternative. | §3.5 |
| 7 | Relationship to graph-monitor | New implementation. Adopt graph-monitor's message definitions, reimplement scraping + reconciliation. | §3.6 |
| 8 | Mixin pattern | `mixins:` section referencing interface fragments. Host's effective interface = own declaration + all mixins merged. | §3.2 |
| 9 | Adoption path | `ros-tooling` org → REP for schema → docs.ros.org tutorials → `ros_core` (long-term). | §4 |
| 10 | Declaration scope | Structural (node interfaces) only for Phase 1–2. Behavioural scope deferred to Phase 3+. | §2 |
