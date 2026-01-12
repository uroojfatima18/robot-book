# Data Model: Chapter 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20
**Purpose**: Define content entities, relationships, and validation rules

## Entity Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                          CHAPTER                                 │
│  id: chapter_1_ros2                                             │
│  title: "The Robotic Nervous System (ROS 2)"                    │
└───────────────────────────────┬─────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
┌───────────────┐       ┌───────────────┐       ┌───────────────┐
│     TIER      │       │     TIER      │       │     TIER      │
│   Beginner    │       │  Intermediate │       │   Advanced    │
└───────┬───────┘       └───────┬───────┘       └───────┬───────┘
        │                       │                       │
   ┌────┴────┐             ┌────┴────┐             ┌────┴────┐
   ▼         ▼             ▼         ▼             ▼         ▼
┌──────┐ ┌──────┐     ┌──────┐ ┌──────┐     ┌──────┐ ┌──────┐
│ L-B1 │ │ L-B2 │     │ L-I1 │ │ L-I2 │     │ L-A1 │ │ L-A2 │
└──┬───┘ └──┬───┘     └──┬───┘ └──┬───┘     └──┬───┘ └──┬───┘
   │        │            │        │            │        │
   ▼        ▼            ▼        ▼            ▼        ▼
┌──────────────────────────────────────────────────────────────┐
│                     CODE EXAMPLES                             │
│                     DIAGRAMS                                  │
│                     AI PROMPTS                                │
└──────────────────────────────────────────────────────────────┘
```

---

## Entity Definitions

### 1. Chapter

The top-level container for all content.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (e.g., `chapter_1_ros2`) |
| title | string | Yes | Display title |
| description | string | Yes | Brief summary (1-2 sentences) |
| level_range | string | Yes | Skill progression (e.g., "Beginner → Advanced") |
| estimated_time | string | Yes | Total completion time (e.g., "6-12 hours") |
| prerequisites | string[] | No | Required prior knowledge |
| learning_outcomes | string[] | Yes | What readers will learn (3-5 outcomes) |
| tiers | Tier[] | Yes | Collection of skill-level groupings |

**Validation Rules**:
- `id` must be lowercase, alphanumeric with underscores
- `title` must be unique across all chapters
- `tiers` must contain exactly 3 entries (beginner, intermediate, advanced)

---

### 2. Tier

A skill-level grouping containing related lessons.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Tier identifier (`beginner`, `intermediate`, `advanced`) |
| name | string | Yes | Display name |
| objectives | string[] | Yes | Learning goals for this tier (3-5 items) |
| estimated_time | string | Yes | Completion time (e.g., "2-4 hours") |
| lessons | Lesson[] | Yes | Ordered list of lessons |
| exercises | Exercise[] | No | Tier-level exercises |

**Validation Rules**:
- `id` must be one of: `beginner`, `intermediate`, `advanced`
- `objectives` must be measurable (action verbs: understand, create, implement)
- `lessons` must be ordered by progression

---

### 3. Lesson

An individual learning unit with content and examples.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Lesson identifier (e.g., `b_lesson1_intro`) |
| title | string | Yes | Display title |
| description | string | Yes | Brief summary (1 sentence) |
| estimated_time | string | Yes | Completion time (e.g., "1-2 hours") |
| content | string | Yes | Markdown content body |
| code_examples | CodeExample[] | Yes | Executable code snippets |
| diagrams | Diagram[] | No | Visual aids |
| ai_prompts | AIPrompt[] | Yes | RAG-compatible prompts |
| prerequisites | string[] | No | Required prior lessons |
| hardware_notes | string | No | Real-world deployment considerations |

**Validation Rules**:
- `id` must follow pattern: `{tier_letter}_lesson{n}_{slug}`
- `code_examples` must contain at least 1 entry
- `ai_prompts` must contain at least 2 entries
- All code must be tested and working

---

### 4. CodeExample

An executable code snippet demonstrating a concept.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Example identifier |
| title | string | Yes | What the code demonstrates |
| language | string | Yes | Programming language (`python`, `bash`, `xml`) |
| code | string | Yes | The actual code |
| expected_output | string | No | What the code produces when run |
| file_path | string | No | Path in repository if standalone file |
| dependencies | string[] | No | Required packages or tools |
| ros2_commands | string[] | No | ROS 2 CLI commands to run |

**Validation Rules**:
- `language` must be one of: `python`, `bash`, `xml`, `yaml`
- `code` must be syntactically valid
- Python code must include proper imports and main guards

---

### 5. Diagram

A visual representation of concepts or architecture.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Diagram identifier |
| title | string | Yes | What the diagram shows |
| alt_text | string | Yes | Accessibility description |
| file_path | string | Yes | Path to SVG file |
| caption | string | No | Extended description |

**Validation Rules**:
- `file_path` must end with `.svg`
- `alt_text` must be descriptive (not just file name)
- File must exist at specified path

---

### 6. AIPrompt

A RAG-compatible prompt for AI-assisted learning.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Prompt identifier |
| category | string | Yes | Prompt type |
| prompt | string | Yes | The question or instruction |
| context | string | No | Background information for AI |
| expected_topics | string[] | Yes | Topics the response should cover |
| tags | string[] | Yes | Indexing tags for RAG |

**Validation Rules**:
- `category` must be one of: `conceptual`, `debugging`, `extension`, `real-world`
- `tags` must include tier identifier
- `expected_topics` must have at least 2 entries

---

### 7. Exercise

A hands-on task for readers to complete.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Exercise identifier |
| title | string | Yes | Exercise name |
| description | string | Yes | What the reader should do |
| difficulty | string | Yes | Relative difficulty |
| acceptance_criteria | string[] | Yes | How to verify completion |
| hints | string[] | No | Progressive hints |
| solution_path | string | No | Path to solution file |

**Validation Rules**:
- `difficulty` must be one of: `easy`, `medium`, `hard`
- `acceptance_criteria` must be testable
- At least one exercise per tier

---

## Relationships

```
Chapter 1──────────┬───────────────────────────────────────────┐
                   │                                           │
                   ▼                                           │
            ┌──────────────┐                                   │
            │    Tier      │◄──────────────────────────────────┤
            │  (1:3)       │                                   │
            └──────┬───────┘                                   │
                   │ contains                                  │
                   ▼                                           │
            ┌──────────────┐                                   │
            │   Lesson     │◄──────────────────────────────────┤
            │  (2-3 per)   │                                   │
            └──────┬───────┘                                   │
                   │ contains                                  │
         ┌─────────┼─────────┬─────────────┐                   │
         ▼         ▼         ▼             ▼                   │
    ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐           │
    │CodeEx   │ │Diagram  │ │AIPrompt │ │Exercise │           │
    │(1+)     │ │(0+)     │ │(2+)     │ │(0+)     │           │
    └─────────┘ └─────────┘ └─────────┘ └─────────┘           │
                                                               │
    Constraints:                                               │
    ─────────────────────────────────────────────────────────────
    • Chapter has exactly 3 Tiers
    • Each Tier has 2-3 Lessons
    • Each Lesson has at least 1 CodeExample
    • Each Lesson has at least 2 AIPrompts
    • Each Tier has at least 1 Exercise
```

---

## Content Instances

### Chapter 1 Structure

```yaml
chapter:
  id: chapter_1_ros2
  title: "The Robotic Nervous System (ROS 2)"
  tiers:
    - id: beginner
      lessons:
        - id: b_lesson1_intro
          title: "Introduction to ROS 2"
          code_examples: [ros2_version, talker_demo]
          diagrams: [ros2_architecture, node_topic_service]
          ai_prompts: [what_is_ros2, ros_vs_ros2, node_definition]

        - id: b_lesson2_sensors
          title: "Basic Sensors Overview"
          code_examples: [sensor_read_simulation]
          diagrams: [humanoid_sensor_placement]
          ai_prompts: [imu_purpose, lidar_vs_camera, force_sensor_use]

    - id: intermediate
      lessons:
        - id: i_lesson1_nodes_topics
          title: "Nodes, Topics, Services, and Actions"
          code_examples: [minimal_publisher, minimal_subscriber, simple_service]
          diagrams: [pub_sub_flow, service_pattern, action_pattern]
          ai_prompts: [topic_vs_service, when_use_action, qos_basics]

        - id: i_lesson2_python_integration
          title: "Python ROS Bridge (rclpy)"
          code_examples: [parameter_demo, launch_file]
          diagrams: [rclpy_architecture]
          ai_prompts: [parameter_types, launch_vs_run, callback_groups]

    - id: advanced
      lessons:
        - id: a_lesson1_urdf_humanoid
          title: "URDF & Humanoid Robot Description"
          code_examples: [basic_urdf, humanoid_skeleton]
          diagrams: [urdf_structure, joint_types]
          ai_prompts: [link_vs_joint, urdf_vs_xacro, collision_geometry]

        - id: a_lesson2_advanced_patterns
          title: "Advanced ROS 2 Patterns & AI Integration"
          code_examples: [fibonacci_action_server, fibonacci_action_client]
          diagrams: [action_lifecycle]
          ai_prompts: [action_feedback, ai_integration_pattern, async_ros2]
```

---

## Validation Summary

| Entity | Count | Validation Status |
|--------|-------|-------------------|
| Chapter | 1 | Defined |
| Tiers | 3 | Beginner, Intermediate, Advanced |
| Lessons | 6 | 2 per tier |
| Code Examples | 15+ | Minimum requirements met |
| Diagrams | 8+ | Required visuals identified |
| AI Prompts | 12+ | Minimum 2 per lesson |
| Exercises | 3+ | Minimum 1 per tier |

**Model Status**: Complete and ready for contract generation.
