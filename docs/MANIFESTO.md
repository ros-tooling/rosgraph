# ROSGraph — Direction

## Why

Robotics engineers spend too much time on ROS plumbing — writing boilerplate, debugging invisible wiring, and keeping launch files in sync with code — instead of building their application.

## What

A declarative, observable ROS graph. Engineers declare what their system should be; tooling generates the code and verifies the running system matches the spec.

## How

1. **Language** — a formal spec to describe node interfaces and system graphs.
2. **Tooling** — translate declarations into working code.
3. **Verification** — compare spec against reality, both at runtime and statically before launch.
