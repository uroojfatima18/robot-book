# Data Model: Chapter 3 - AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-27
**Feature**: 003-isaac-ai-brain

## Entity Overview

This document defines the content entities for Chapter 3, their attributes, relationships, and validation rules.

---

## Core Entities

### 1. Chapter

The top-level container for all Chapter 3 content.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique identifier: "003" |
| title | string | Yes | "AI-Robot Brain (NVIDIA Isaac)" |
| description | string | Yes | Brief chapter overview |
| learningOutcomes | string[] | Yes | List of 6 learning outcomes |
| prerequisites | Reference[] | Yes | Links to Chapter 1 and 2 |
| tiers | Tier[] | Yes | Exactly 3 tiers |

**Validation Rules**:
- Must have exactly 3 tiers (Beginner, Intermediate, Advanced)
- Prerequisites must reference valid chapter IDs
- Learning outcomes must be measurable

---

### 2. Tier

A skill-level grouping within the chapter.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | enum | Yes | "beginner" | "intermediate" | "advanced" |
| displayName | string | Yes | Human-readable name |
| objectives | string[] | Yes | 3-5 learning objectives |
| estimatedHours | number | Yes | 3-5 hours expected |
| lessons | Lesson[] | Yes | Ordered list of lessons |
| exercises | Exercise[] | Yes | Tier-specific exercises |

**Validation Rules**:
- Beginner tier must have no external prerequisites
- Intermediate tier requires Beginner completion
- Advanced tier requires Intermediate completion
- Each tier must have 2-5 lessons

**State Transitions**:
```
[Not Started] -> [In Progress] -> [Completed]
                      |
                      v
               [Needs Review]
```

---

### 3. Lesson

An individual learning unit with theory, code, and exercises.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique ID: "B1", "I2", "A3", etc. |
| title | string | Yes | Lesson title |
| tier | enum | Yes | Parent tier reference |
| prerequisites | string[] | No | Lesson IDs required before this |
| sections | Section[] | Yes | Ordered content sections |
| codeExamples | CodeExample[] | Yes | 1-3 code examples |
| diagrams | Diagram[] | No | Visual aids |
| aiPrompts | AIPrompt[] | Yes | 2-3 RAG prompts |
| summary | string | Yes | Key takeaways |

**Section Structure** (per NFR-005):
1. Theory
2. Code Example
3. Hands-on Exercise
4. Summary

---

### 4. CodeExample

Executable Python/ROS 2 code snippet.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique identifier |
| language | enum | Yes | "python" | "yaml" | "xml" | "bash" |
| filename | string | Yes | Suggested filename |
| code | string | Yes | Actual code content |
| expectedOutput | string | No | What reader should see |
| rosMessages | string[] | No | ROS 2 message types used |
| dependencies | string[] | Yes | Required packages |
| annotations | Annotation[] | No | Line-by-line explanations |

**Validation Rules**:
- Python code must pass syntax check
- YAML must be valid
- Dependencies must be installable via apt or pip

---

### 5. Diagram

Visual representation of concepts.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique identifier |
| type | enum | Yes | "flowchart" | "architecture" | "comparison" | "sequence" |
| format | enum | Yes | "svg" | "mermaid" | "png" |
| altText | string | Yes | Screen reader description (NFR-001) |
| caption | string | Yes | Figure caption |
| colorScheme | string | Yes | Must be color-blind safe (NFR-002) |
| source | string | Yes | Path to diagram file or Mermaid code |

**Color Palette Constraint**:
- Primary: #0072B2 (blue)
- Secondary: #E69F00 (orange)
- Success: #009E73 (teal)
- Warning: #F0E442 (yellow)
- Error: #D55E00 (vermillion)

---

### 6. AIPrompt

RAG-compatible prompt for learner assistance.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique identifier |
| type | enum | Yes | "concept" | "debug" | "extension" |
| prompt | string | Yes | The actual prompt text |
| context | string | Yes | Lesson context for RAG |
| expectedTopics | string[] | Yes | Topics the response should cover |

---

### 7. Exercise

Hands-on practice activity.

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| id | string | Yes | Unique identifier |
| title | string | Yes | Exercise name |
| tier | enum | Yes | Difficulty level |
| description | string | Yes | What to do |
| objectives | string[] | Yes | Learning goals |
| starterCode | string | No | Optional starting point |
| solution | string | No | Reference solution |
| hints | string[] | No | Progressive hints |
| validationSteps | string[] | Yes | How to verify success |

---

## Entity Relationships

```
Chapter (1) ─────> (3) Tier
                        │
                        ├── (2-5) Lesson
                        │         │
                        │         ├── (1-3) CodeExample
                        │         ├── (0-3) Diagram
                        │         └── (2-3) AIPrompt
                        │
                        └── (2-5) Exercise
```

---

## Lesson Inventory

### Beginner Tier (3 lessons)
| ID | Title | FR Reference |
|----|-------|--------------|
| B1 | Introduction to Robotic Perception | FR-002 |
| B2 | Understanding Sensor Types | FR-003 |
| B3 | SLAM and Navigation Concepts | FR-004, FR-005 |

### Intermediate Tier (4 lessons)
| ID | Title | FR Reference |
|----|-------|--------------|
| I1 | Camera and Depth Data Processing | FR-008, FR-009 |
| I2 | TF2 Coordinate Frames | FR-010 |
| I3 | SLAM Toolbox Configuration | FR-011 |
| I4 | Nav2 Basics | FR-012, FR-015 |

### Advanced Tier (4 lessons)
| ID | Title | FR Reference |
|----|-------|--------------|
| A1 | Costmap Configuration | FR-016 |
| A2 | Planners and Behavior Trees | FR-017, FR-018 |
| A3 | Reinforcement Learning Fundamentals | FR-019, FR-020 |
| A4 | Sim-to-Real Transfer | FR-021, FR-022, FR-023 |

---

## Diagram Inventory

| ID | Title | Lesson | Type |
|----|-------|--------|------|
| D1 | Perception Pipeline | B1 | flowchart |
| D2 | Sensor Comparison Matrix | B2 | comparison |
| D3 | Navigation Architecture | B3 | architecture |
| D4 | TF Tree Example | I2 | architecture |
| D5 | SLAM Process Flow | I3 | flowchart |
| D6 | Costmap Layers | A1 | architecture |
| D7 | RL Training Loop | A3 | flowchart |
| D8 | Sim-to-Real Gap | A4 | comparison |

**Total**: 8 diagrams across 3 tiers
