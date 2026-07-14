# Roadmap

## Current scope
- Real Piper control unit in pure Python
- Shared mock mode for demo and testing
- Pluggable policy interface
- Remote policy adapters over websocket and HTTP

## Non-goals for v1
- Full ROS2 Jazzy package
- Training pipeline
- Complex GUI
- Multi-model orchestration
- Cartesian control as the main interface

## Next stage
- Add a ROS2 bridge layer on Ubuntu 24 / Jazzy
- Replace the placeholder real driver hooks with the exact official SDK wiring from the target hardware setup
- Add camera-backed observations for OpenPI or Pi0.5 style policies
