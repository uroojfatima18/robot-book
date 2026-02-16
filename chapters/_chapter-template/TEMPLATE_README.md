# Chapter Template Guide

This directory contains templates for creating new chapters in the Physical AI & Humanoid Robotics textbook. Use these files as a starting point for Chapters 2-6.

---

## Template Overview

### Files in This Template

```
_chapter-template/
├── README.md                          # Main chapter landing page
├── introduction.md                    # Chapter introduction (5-15 min read)
├── glossary.md                        # Terminology reference
├── summary.md                         # Chapter summary and key takeaways
│
├── beginner/
│   ├── README.md                      # Beginner tier overview
│   ├── 01-lesson-title.md             # Lesson 1 (copy/modify LESSON_TEMPLATE.md)
│   ├── 02-lesson-title.md             # Lesson 2
│   └── exercises/
│       └── beginner-exercises.md      # All beginner tier exercises
│
├── intermediate/
│   ├── README.md                      # Intermediate tier overview
│   ├── 01-lesson-title.md             # Lesson 1
│   ├── 02-lesson-title.md             # Lesson 2
│   └── exercises/
│       └── intermediate-exercises.md  # All intermediate tier exercises
│
├── advanced/
│   ├── README.md                      # Advanced tier overview
│   ├── 01-lesson-title.md             # Lesson 1
│   ├── 02-lesson-title.md             # Lesson 2
│   └── exercises/
│       └── advanced-exercises.md      # All advanced tier exercises
│
├── ai-prompts/
│   ├── beginner-prompts.md            # AI prompts for beginners
│   ├── intermediate-prompts.md        # AI prompts for intermediates
│   └── advanced-prompts.md            # AI prompts for advanced learners
│
├── code/
│   ├── beginner/                      # Beginner tier code examples
│   ├── intermediate/                  # Intermediate tier code examples
│   └── advanced/                      # Advanced tier code examples
│
├── diagrams/                          # Visual aids (PNG, SVG)
│   └── [diagram files]
│
├── LESSON_TEMPLATE.md                 # Template for individual lessons
├── TEMPLATE_README.md                 # This file
└── README.md                          # Chapter main page (use as template)
```

---

## How to Use These Templates

### Step 1: Copy the Template Directory

```bash
# Copy template to create your new chapter
cp -r chapters/_chapter-template chapters/0[X]-[chapter-slug]

# Example for Chapter 2:
cp -r chapters/_chapter-template chapters/02-digital-twin
```

### Step 2: Customize the Chapter Template Files

#### README.md (Main Chapter Page)
```markdown
1. Replace [X] with chapter number (2, 3, 4, 5, or 6)
2. Replace [Chapter Title] with your chapter's title
3. Update the "Three-Tier Learning Structure" section with actual tier focuses
4. Update the "Complete Table of Contents" with your actual lessons
5. Update prerequisites and technical requirements
6. Replace estimated time with your values
```

#### introduction.md
```markdown
1. Replace chapter ID and title
2. Write compelling opening hook (why this matters)
3. Explain core concepts preview
4. Update hardware notes for your chapter's focus
5. Update prerequisites checklist
6. Describe what learners will build
```

#### glossary.md
```markdown
1. Replace chapter ID
2. List all key terms from your chapter
3. Provide clear, robotics-focused definitions
4. Include examples for each term
5. Create a table of abbreviations
6. Clarify common confusions
```

#### summary.md
```markdown
1. Replace chapter ID
2. Summarize what learners will have learned in each tier
3. Create review questions (beginner, intermediate, advanced)
4. List what they've built
5. Describe common pitfalls
6. Preview next chapter
```

### Step 3: Customize Tier Templates

#### beginner/README.md
```markdown
1. Replace placeholders [X]-[X] with actual hours
2. Update Learning Objectives for your chapter's Beginner tier
3. Update Prerequisites
4. Create Lesson B1 and B2 sections with your lessons
5. Update exercises and AI prompts references
```

#### intermediate/README.md
```markdown
1. Replace placeholders with actual content
2. Update Learning Objectives
3. Describe Lessons I1 and I2
4. Update best practices for your tier
5. Update code examples location
6. Update resources
```

#### advanced/README.md
```markdown
1. Replace placeholders with your content
2. Update Learning Objectives (these should be ambitious)
3. Describe Lessons A1 and A2
4. Add advanced concepts covered
5. Describe connections to later chapters
6. Update resources with advanced references
```

### Step 4: Create Individual Lessons

For each lesson, use LESSON_TEMPLATE.md:

```bash
# Copy the lesson template
cp chapters/_chapter-template/LESSON_TEMPLATE.md chapters/0[X]-[chapter-slug]/beginner/01-lesson-title.md
```

Then customize:
```markdown
1. Update the frontmatter (id, title, tier, chapter, estimated_time, prerequisites)
2. Update Learning Objectives
3. Write the Introduction
4. Create Sections 1, 2, 3 with theories, examples, and practices
5. Add a Putting It Together section
6. Create Practical Challenges or Deep Dives
7. Write Summary with Key Takeaways
8. Add Check Your Understanding questions
9. Reference exercises and further reading
```

### Step 5: Create Exercises

For each tier, create an exercises file:

```markdown
beginner/exercises/beginner-exercises.md
intermediate/exercises/intermediate-exercises.md
advanced/exercises/advanced-exercises.md
```

Each should include:
- Exercise title and description
- What the learner needs to do
- Expected output or acceptance criteria
- Solution hints or guidance
- Links to relevant lessons

### Step 6: Create AI Prompts

In ai-prompts/ directory, create three files with example prompts for each tier:

```markdown
ai-prompts/beginner-prompts.md    # Conceptual prompts
ai-prompts/intermediate-prompts.md # Implementation prompts
ai-prompts/advanced-prompts.md    # Architecture prompts
```

Each prompt file should include:
- Category of prompts (e.g., "Conceptual Questions", "Debugging")
- 5-10 example prompts learners can use
- Instructions on how to modify/extend prompts

### Step 7: Add Code Examples

Create working code examples in the code/ directory:

```
code/
├── beginner/
│   └── demo_or_intro.sh
├── intermediate/
│   ├── example1.py
│   ├── example2.py
│   └── README.md
└── advanced/
    ├── pattern1.py
    ├── pattern2.py
    └── README.md
```

All code should:
- Be production-ready (or clearly marked as learning examples)
- Include comments and docstrings
- Have a README explaining what each file does
- Be testable and runnable
- Include error handling

### Step 8: Add Diagrams

Create visual aids in the diagrams/ directory:

```
diagrams/
├── concept-diagram.svg
├── architecture-overview.png
├── flow-chart.svg
└── comparison-table.png
```

For each diagram, add a description comment in the relevant lesson explaining what the diagram shows.

---

## Key Principles to Remember

### 1. Progressive Mastery

Each tier builds on the previous:
- **Beginner**: Concepts and understanding (no coding)
- **Intermediate**: Hands-on implementation and practice
- **Advanced**: Theory, patterns, and production considerations

### 2. No Assumed Knowledge

Each chapter should start from zero in the Beginner tier. Even if Chapter 2 depends on Chapter 1, the Beginner tier of Chapter 2 should be accessible to someone who just finished Chapter 1.

### 3. Textbook Feel

This should feel like a professional textbook, not API documentation:
- Use metaphors and analogies
- Tell stories and provide context
- Show why things matter, not just how to use them
- Progress logically and clearly

### 4. AI-Native

Content should be:
- Queryable by AI chatbots
- RAG-compatible (good structure for retrieval)
- Machine-readable (clear formatting, code blocks)
- Personalization-friendly (hooks for learner background)

### 5. Hands-On & Practical

- Every lesson should have code, exercises, or hands-on practice
- Provide working examples
- Include troubleshooting guides
- Give learners clear acceptance criteria

---

## Customization Checklist

Before considering your chapter complete:

### Main Chapter Files
- [ ] README.md customized with chapter title, tiers, and table of contents
- [ ] introduction.md written with compelling hook and context
- [ ] glossary.md populated with all key terms
- [ ] summary.md written with key takeaways and review questions

### Tier Overview Files
- [ ] beginner/README.md updated with B1, B2 lesson descriptions
- [ ] intermediate/README.md updated with I1, I2 lesson descriptions
- [ ] advanced/README.md updated with A1, A2 lesson descriptions

### Individual Lessons
- [ ] At least 2 lessons per tier (6 total)
- [ ] Each lesson uses consistent structure (LESSON_TEMPLATE.md)
- [ ] Learning objectives are clear and measurable
- [ ] Examples are concrete and robotics-relevant

### Exercises & Support
- [ ] beginner-exercises.md with 3-5 exercises
- [ ] intermediate-exercises.md with 4-6 exercises + checkpoint project
- [ ] advanced-exercises.md with 5-7 exercises + capstone project
- [ ] beginner-prompts.md with 5-10 AI prompts
- [ ] intermediate-prompts.md with 5-10 AI prompts
- [ ] advanced-prompts.md with 5-10 AI prompts

### Code & Visuals
- [ ] Code examples in code/ directory with README
- [ ] Diagrams in diagrams/ directory with descriptions
- [ ] Links to diagrams from lessons

### Constitution Compliance
- [ ] Chapter follows the Book Constitution principles
- [ ] No prerequisites beyond previous chapters (for Beginner tier)
- [ ] Each tier builds on previous (Beginner → Intermediate → Advanced)
- [ ] Code examples are ROS 2 + Python (or whatever your chapter focus is)
- [ ] Simulation-first approach where applicable
- [ ] AI prompts included for all tiers
- [ ] Safety considerations addressed (if applicable)

---

## Template Modification Examples

### Example 1: Chapter 2 - The Digital Twin

```bash
# Copy template
cp -r chapters/_chapter-template chapters/02-digital-twin

# Customize README.md
# - Chapter: The Digital Twin (Gazebo & Unity)
# - Beginner: Understand what simulation is
# - Intermediate: Build a world in Gazebo
# - Advanced: High-fidelity visualization in Unity

# Customize lessons
# B1: What is a Digital Twin?
# B2: Simulation vs. Reality
# I1: Gazebo Basics and Physics
# I2: Sensor Simulation in Gazebo
# A1: Unity for High-Fidelity Visualization
# A2: Bridging Gazebo and Unity
```

### Example 2: Chapter 3 - Perception & Sensors

```bash
# Copy template
cp -r chapters/_chapter-template chapters/03-perception-sensors

# Customize for focus on sensor processing
# B1: Types of Sensors and Their Data
# B2: Sensor Data Visualization
# I1: Processing Camera Data (OpenCV)
# I2: LIDAR and Point Clouds
# A1: Sensor Fusion Techniques
# A2: ROS 2 Perception Pipelines
```

---

## Questions or Issues?

If you need to adapt these templates for your chapter:

1. **Check the constitution** (`constitution.md`) for requirements
2. **Reference Chapter 1** (`chapters/01-ros2-nervous-system/`) for examples of completed chapters
3. **Review the plan** (`plan.md`) for your chapter's specific requirements
4. **Modify templates** as needed while maintaining the 3-tier structure and textbook feel

---

## Final Notes

These templates are designed to:
- Speed up chapter creation
- Ensure consistency across all chapters
- Maintain quality and progressive structure
- Support the book's AI-native, simulation-first philosophy

Modify them as needed for your specific chapter content, but keep the overall structure and pedagogical approach intact.

**Happy authoring! The book gets better with each new chapter you create.**
