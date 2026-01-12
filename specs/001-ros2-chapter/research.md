# Research: Chapter 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20
**Purpose**: Resolve technical unknowns and establish best practices for chapter content

## Executive Summary

This research document consolidates decisions for implementing Chapter 1 of the Physical AI & Humanoid Robotics textbook. All technical unknowns have been resolved with rationale and alternatives documented.

---

## 1. ROS 2 Distribution Selection

### Decision
**Primary**: ROS 2 Humble Hawksbill (LTS)
**Secondary**: ROS 2 Iron Irwini (latest stable)

### Rationale
- Humble is Long Term Support (LTS) until May 2027
- Widest adoption in industry and education as of 2025
- Iron provides access to newest features for advanced readers
- Both run on Ubuntu 22.04 LTS, our target platform

### Alternatives Considered
| Distribution | Considered | Rejected Because |
|--------------|------------|------------------|
| Jazzy Jalisco | Yes | Too new (2024), less documentation, Ubuntu 24.04 required |
| Galactic | No | End-of-life November 2022 |
| Foxy Fitzroy | Yes | End-of-life May 2023, outdated |

### Implementation Notes
- All code examples tested on Humble first
- Iron-specific features marked with version badges
- Deprecation warnings included for version-sensitive APIs

---

## 2. Installation Approach

### Decision
**Binary installation via apt packages** (not source build)

### Rationale
- Simplest path for beginners with no prior ROS experience
- Consistent results across reader systems
- Faster setup (minutes vs hours for source build)
- Official packages maintained by Open Robotics

### Alternatives Considered
| Method | Considered | Rejected Because |
|--------|------------|------------------|
| Source build | Yes | Too complex for beginners, error-prone |
| Docker | Yes | Adds container complexity, GUI challenges |
| Snap | Yes | Limited package availability |

### Implementation Notes
```bash
# Installation commands to document:
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

---

## 3. Robot Description Format

### Decision
**URDF (Unified Robot Description Format)** for Chapter 1
**XACRO mentioned as future topic** (later chapters)

### Rationale
- URDF is the foundational format, must be understood first
- XACRO adds macro complexity better suited for production robots
- Beginner-friendly: pure XML, no preprocessing required
- All ROS 2 tools support URDF natively

### Alternatives Considered
| Format | Considered | Rejected Because |
|--------|------------|------------------|
| XACRO | Yes | Adds preprocessing complexity for beginners |
| SDF (Gazebo) | Yes | Simulation-specific, not ROS 2 native |
| MJCF (MuJoCo) | No | Different ecosystem, not ROS 2 standard |

### Implementation Notes
- Basic URDF structure with links and joints
- Humanoid example: base_link → torso → head, arms, legs
- Visual/collision geometry with simple shapes
- RViz2 visualization for immediate feedback

---

## 4. Simulation Environment

### Decision
**Gazebo Fortress** (recommended)
**Gazebo Classic** (supported for legacy systems)

### Rationale
- Gazebo Fortress is the current "modern Gazebo" release
- Better performance and modern architecture
- ROS 2 integration via ros_gz packages
- Classic still works for readers with older setups

### Alternatives Considered
| Simulator | Considered | Rejected Because |
|-----------|------------|------------------|
| NVIDIA Isaac Sim | Yes | Chapter 4 topic, too advanced for intro |
| Unity Robotics | Yes | Chapter 3 topic, different focus |
| Webots | No | Less common in ROS 2 ecosystem |
| PyBullet | No | Not standard ROS 2 integration |

### Implementation Notes
- Gazebo launch from ROS 2 launch files
- Sensor simulation: cameras, LIDAR, IMU
- Simple humanoid model for visualization
- Physics not deeply covered (Chapter 3 topic)

---

## 5. Code Organization Strategy

### Decision
**Standalone Python scripts** (not ROS 2 packages)

### Rationale
- Chapter focuses on ROS 2 concepts, not colcon/build systems
- Readers can run examples immediately without package creation
- Package structure is a later-chapter topic
- Reduces barrier to entry for beginners

### Alternatives Considered
| Approach | Considered | Rejected Because |
|----------|------------|------------------|
| Full ROS 2 packages | Yes | Adds colcon/ament complexity too early |
| Hybrid (some packages) | Yes | Inconsistent experience for readers |

### Implementation Notes
- Each script is self-contained and runnable
- Shebang lines: `#!/usr/bin/env python3`
- Source ROS 2 underlay before running
- Package creation covered in later chapters

---

## 6. AI Integration Pattern

### Decision
**ROS 2 Action Client pattern** for AI agent communication

### Rationale
- Actions are designed for long-running tasks (AI inference qualifies)
- Built-in feedback mechanism for progress updates
- Cancellation support for user control
- Standard ROS 2 pattern, not custom protocol

### Alternatives Considered
| Pattern | Considered | Rejected Because |
|---------|------------|------------------|
| Services | Yes | No feedback, blocks during inference |
| Topics | Yes | No request-response semantics |
| Custom gRPC | No | Not ROS 2 native, adds complexity |

### Implementation Notes
- Fibonacci example demonstrates action pattern
- AI integration is conceptual in Chapter 1
- Practical AI actions in Chapter 4/5
- Action server handles async requests from AI agents

---

## 7. Diagram Standards

### Decision
**SVG format** with consistent styling

### Rationale
- Scalable for print and web
- Editable in standard tools (Inkscape, Illustrator)
- Text remains selectable for accessibility
- Smaller file sizes than raster formats

### Style Guide
- Colors: ROS 2 blue (#22A7E0), Python yellow (#FFD43B), Safety red (#E74C3C)
- Fonts: Sans-serif (Open Sans or system default)
- Line weight: 2px for connectors, 3px for emphasis
- Background: White or transparent

### Diagrams Needed
1. `ros2-architecture.svg` - DDS middleware, nodes, topics overview
2. `node-topic-service.svg` - Communication patterns
3. `humanoid-sensor-placement.svg` - Sensor locations on humanoid
4. `urdf-structure.svg` - Link/joint tree visualization

---

## 8. AI-Assisted Prompts (RAG)

### Decision
**Markdown files with structured prompts per tier**

### Rationale
- Constitution requires AI Agent Assisted Prompts
- Markdown is machine-readable and RAG-compatible
- Prompts organized by learning tier for relevance
- Can be embedded in chatbot or used standalone

### Prompt Categories
| Category | Example Prompt |
|----------|----------------|
| Conceptual | "Explain the difference between ROS 2 topics and services" |
| Debugging | "My publisher node isn't sending messages. What should I check?" |
| Extension | "How would I add a custom message type to this example?" |
| Real-world | "What sensors would I need for a humanoid to navigate a room?" |

### Implementation Notes
- 5-10 prompts per tier (beginner, intermediate, advanced)
- Prompts include expected context and sample responses
- Tagged for RAG indexing: `#ros2 #beginner #nodes`

---

## 9. Non-Ubuntu Platform Support

### Decision
**Document alternatives, don't officially support**

### Rationale
- Ubuntu 22.04 is the reference platform per specification
- Full multi-platform testing is out of scope
- Alternatives exist and should be mentioned for accessibility

### Documented Alternatives
| Platform | Solution | Notes |
|----------|----------|-------|
| Windows | WSL2 + Ubuntu 22.04 | Near-native experience |
| macOS | Docker + Ubuntu 22.04 | GUI requires X11 forwarding |
| Other Linux | Native packages (Debian, Fedora) | Community-supported |

### Implementation Notes
- Installation section mentions alternatives
- Troubleshooting section covers common WSL2 issues
- Docker Compose file provided in repository

---

## 10. Testing & Validation

### Decision
**Manual validation on reference system** + **CI for code syntax**

### Rationale
- Code examples must run, not just compile
- Manual testing catches UX issues
- CI catches typos and syntax errors
- Full ROS 2 CI too complex for educational content

### Validation Checklist
- [ ] All code examples run on Ubuntu 22.04 + ROS 2 Humble
- [ ] All code examples run on Ubuntu 22.04 + ROS 2 Iron
- [ ] URDF loads in RViz2 without errors
- [ ] Gazebo simulations launch successfully
- [ ] AI prompts produce relevant responses

---

## Resolved Unknowns Summary

| Unknown | Resolution | Confidence |
|---------|------------|------------|
| ROS 2 version | Humble LTS + Iron | High |
| Installation method | apt packages | High |
| Robot format | URDF (not XACRO) | High |
| Simulator | Gazebo Fortress | High |
| Code organization | Standalone scripts | High |
| AI pattern | Action client | High |
| Diagram format | SVG | High |
| Non-Ubuntu support | Documented alternatives | Medium |

**All NEEDS CLARIFICATION items resolved. Ready for Phase 1.**
