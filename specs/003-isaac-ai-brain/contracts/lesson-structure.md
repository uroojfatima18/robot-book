# Contract: Lesson Structure

**Date**: 2025-12-27
**Feature**: 003-isaac-ai-brain

## Purpose

This contract defines the required structure for all lessons in Chapter 3, ensuring consistency and compliance with NFR-005.

---

## Lesson Template

Every lesson MUST follow this structure:

```markdown
# [Lesson ID]: [Lesson Title]

**Tier**: [Beginner | Intermediate | Advanced]
**Prerequisites**: [List or "None"]
**Estimated Time**: [X minutes]

## Learning Objectives

By the end of this lesson, you will be able to:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Prerequisite Refresher

> [1-2 paragraph inline refresher for Chapter 1/2 concepts referenced]
> (Required by NFR-006)

---

## Theory

[2-4 paragraphs explaining the concept]

### Key Concepts

- **[Term 1]**: Definition
- **[Term 2]**: Definition

### [Diagram if applicable]

![Alt text describing the diagram](./diagrams/diagram-name.svg)
*Figure X: Caption describing what the diagram shows*

---

## Code Example

### [Example Title]

**File**: `code/example_name.py`

```python
#!/usr/bin/env python3
# Code with inline comments
```

**Expected Output**:
```
[What the reader should see when running]
```

**ROS 2 Messages Used**:
- `sensor_msgs/msg/Image`
- `geometry_msgs/msg/Twist`

---

## Hands-on Exercise

### Exercise: [Exercise Title]

**Objective**: [What the reader will accomplish]

**Steps**:
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Validation**:
- [ ] [Checkable outcome 1]
- [ ] [Checkable outcome 2]

**Hints** (if stuck):
<details>
<summary>Hint 1</summary>
[First hint]
</details>

---

## Summary

### Key Takeaways
- [Takeaway 1]
- [Takeaway 2]
- [Takeaway 3]

### What's Next
[Brief preview of the next lesson]

---

## AI Agent Prompts

**Concept Check**:
> "Explain how [concept] works in robotic perception/navigation"

**Debugging Help**:
> "I'm getting [common error]. How do I fix it?"

**Extension**:
> "How would I extend this to [advanced use case]?"
```

---

## Validation Checklist

For each lesson, verify:

- [ ] Title follows pattern: "[ID]: [Title]"
- [ ] Tier is correctly specified
- [ ] Prerequisites are listed or "None"
- [ ] Learning objectives are measurable (3-5 items)
- [ ] Prerequisite refresher included (if referencing Ch1/Ch2)
- [ ] Theory section present with 2-4 paragraphs
- [ ] At least 1 diagram with alt-text (if applicable)
- [ ] Code example is complete and runnable
- [ ] Expected output documented
- [ ] Hands-on exercise has clear steps
- [ ] Validation criteria are checkable
- [ ] Summary contains 3 key takeaways
- [ ] 2-3 AI Agent Prompts included
