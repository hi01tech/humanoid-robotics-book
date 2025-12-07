# Data Model for Textbook Content

**Date**: 2025-12-07
**Associated Plan**: [specs/main/plan.md](specs/main/plan.md)

This document defines the data structures for the content artifacts of the "Physical AI & Humanoid Robotics" textbook. The primary entity is the content file, which is represented by its YAML frontmatter metadata.

## Content File Entity (`*.md` / `*.mdx`)

This is the core entity representing any piece of educational content in the textbook. Its properties are defined by the YAML frontmatter required by FR-011.

### Properties

| Field | Type | Description | Required | Example |
|---|---|---|---|---|
| `id` | String | A unique identifier for the document, used for URL generation. | Yes | `ros2-week4` |
| `title` | String | The main title of the document, displayed at the top of the page. | Yes | `Introduction to ROS 2` |
| `sidebar_label` | String | The short label used for this document in the navigation sidebar. | Yes | `ROS 2 Intro` |
| `estimated_time`| Number | The estimated time in hours required to complete the chapter. | Yes | `2.5` |
| `week` | Number | The course week this content corresponds to (1-13). | Yes | `4` |
| `module` | String | The name of the course module this content belongs to. | Yes | `ROS 2` |
| `prerequisites` | Array[String] | A list of `id`s of other documents that are prerequisites for this one. | Yes (can be empty `[]`) | `['kinematics', 'real-time-control']` |
| `learning_objectives` | Array[String] | A list of key learning outcomes for the chapter. | Yes | `["Understand ROS 2 nodes and topics", "Write a simple publisher and subscriber"]` |

### Relationships

-   **Belongs to one `Module`**: The `module` field links a content file to a specific course module.
-   **Corresponds to one `Week`**: The `week` field links a content file to a specific week in the curriculum.
-   **Can have many `prerequisites`**: The `prerequisites` array defines a many-to-many relationship between content files.

### Validation Rules

-   All fields listed as "Required" must be present in the frontmatter of every content file.
-   `id` must be unique across all content files.
-   `week` must be an integer between 1 and 13.
-   `module` must be one of the pre-defined module names (e.g., "Foundations", "ROS 2", "Digital Twin", "NVIDIA Isaac", "VLA").

### Example

```yaml
---
id: ros2-week4
title: Introduction to ROS 2
sidebar_label: ROS 2 Intro
estimated_time: 2.5
week: 4
module: ROS 2
prerequisites: []
learning_objectives:
  - "Understand ROS 2 nodes and topics"
  - "Write a simple publisher and subscriber"
---

... content starts here ...
```

## Other Entities

The other entities identified in the specification (Module, Week, Setup Guide, etc.) are logical groupings or specific types of the `Content File` entity. They are distinguished by their location in the file system (e.g., `docs/setup/`) and their metadata (e.g., `module: 'ROS 2'`), rather than being distinct data structures.
