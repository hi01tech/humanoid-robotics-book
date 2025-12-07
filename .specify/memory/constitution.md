<!-- Sync Impact Report -->
<!-- Version change: N/A (Initial Creation/Major Update to 1.0.0) -->
<!-- List of modified principles: All principles added/defined -->
<!-- Added sections: Key Standards, Constraints, Governance (detailed) -->
<!-- Removed sections: None -->
<!-- Templates requiring updates: -->
<!-- - .specify/templates/plan-template.md: ✅ updated -->
<!-- - .specify/templates/spec-template.md: ✅ updated -->
<!-- - .specify/templates/tasks-template.md: ✅ updated -->
<!-- - .claude/commands/sp.adr.md: ✅ updated -->
<!-- - .claude/commands/sp.analyze.md: ✅ updated -->
<!-- - .claude/commands/sp.checklist.md: ✅ updated -->
<!-- - .claude/commands/sp.clarify.md: ✅ updated -->
<!-- - .claude/commands/sp.constitution.md: ✅ updated -->
<!-- - .claude/commands/sp.git.commit_pr.md: ✅ updated -->
<!-- - .claude/commands/sp.implement.md: ✅ updated -->
<!-- - .claude/commands/sp.phr.md: ✅ updated -->
<!-- - .claude/commands/sp.plan.md: ✅ updated -->
<!-- - .claude/commands/sp.specify.md: ✅ updated -->
<!-- - .claude/commands/sp.tasks.md: ✅ updated -->
<!-- Follow-up TODOs: -->
<!-- - TODO(RATIFICATION_DATE): Original adoption date unknown -->
<!-- End Sync Impact Report -->
# AI-Native Textbook for Physical AI & Humanoid Robotics (Hackathon I) Constitution

## Core Principles

### I. Spec-Driven Creation (Compliance)
Every chapter, feature, and workflow must strictly follow the Spec-Kit Plus workflow (Spec > Clarify > Plan > Tasks > Implement).

### II. AI-Native Authorship
Claude Code subagents must be used for drafting, refining, validating, and maintaining book content.

### III. Technical Accuracy
All robotics, simulation, and AI details must be verifiable against current, official documentation (ROS 2, Gazebo, NVIDIA Isaac Sim 2023+, Unity and OpenAI, documentation).

### IV. Educational Clarity
Content must be simple, clear, and example-based, focused on progressive learning for beginner to intermediate robotics learners.

### V. AI-Augmented Learning
The RAG chatbot must answer questions **ONLY** grounded in the book's text content.

### VI. Reusable Intelligence
Prioritize the creation and documentation of reusable Subagents/Skills for recurring tasks (e.g., Docusaurus file headers, ROS 2 code snippets).

## Key Standards

- **Written Format:** All content must be written in **MDX** for Docusaurus 3+.
- **Content Completeness:** Each chapter must include clear objectives, prerequisites, diagrams, step-by-step workflows, and runnable/validated code examples.
- **Book Structure:** Content must be logically organized into the **4 distinct course modules** (ROS 2, Digital Twin, Isaac Sim, VLA).
- **Source Verification:** All concepts must be traceable to primary research or official vendor documentation.
- **Reading Level & Tone:** Content must maintain a **Flesch-Kincaid Grade 9–12** reading level and a **technical, concise, and instructional** tone.
- **Chatbot Implementation:** The RAG chatbot must utilize a **free-tier architecture** and be built with modular code (e.g., API backend for processing).

## Constraints

- **Deliverable Minimums:** The final book must contain a minimum of **12 chapters** aligned with the 4 course modules(ROS 2 → Gazebo → Unity → Isaac → VLA), **30+ code examples**, **5 lab workflows**, and **1 capstone tutorial**.
- **Tooling:** Only official, supported tooling is allowed; no experimental APIs or hallucinated commands.
- **Personalization (Requirement Gate):** The final solution must support UI elements for personalization features (e.g., "Personalize Chapter" and "Translate to Urdu" buttons), whose implementation will be fully defined in the **Specify Phase**.
- **Platform Integrity:** Must be implemented in **Docusaurus** (Markdown + React/Typescript). Docusaurus builds must succeed without errors.
- **Deployment:** Final output must be deployable to **GitHub Pages**.
- **Integrated Code:** All code snippets must be testable and properly formatted for Docusaurus code block components.
- **Resource Efficiency:** The RAG and customization pipeline must utilize **minimal GPU usage** and **minimal embeddings** to adhere to the free-tier principle.
- **RAG Chatbot Requirement:** Chatbot must be functionally integrated, embedded in UI, capable of answering questions based *only* on the book's text content.

## Governance

This constitution outlines the fundamental principles and operational guidelines for the AI-Native Textbook project. Adherence to these rules is mandatory for all contributors and subagents. Amendments require careful documentation, stakeholder approval, and a clear migration plan for affected components.

**Success Criteria:**
- The entire project demonstrates adherence to the **Spec-Kit Plus workflow**.
- The Docusaurus site builds successfully and smoothly deploys to GitHub Pages.
- All core content modules are complete, technically accurate, and verifiable.
- The integrated RAG Chatbot is functional, polite, and answers questions correctly **only** using the book's content.
- Code (for RAG and personalization UI) is modular, secure, and adheres to modern standards.
- Clear **Architecture Decision Records (ADRs)** exist for all major architectural choices (e.g., database, API backend).

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2025-12-06
