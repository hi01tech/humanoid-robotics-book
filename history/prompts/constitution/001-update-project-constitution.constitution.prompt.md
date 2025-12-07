---
id: 001
title: Update project constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- none
---

## Prompt

```text
Project: AI-Native Textbook for Physical AI & Humanoid Robotics (Hackathon I)

Target audience: Students, educators, and developers learning ROS 2, Gazebo, Unity, NVIDIA Isaac, and embodied AI systems

Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.


Core principles:
- **Spec-Driven Creation (Compliance):** Every chapter, feature, and workflow must strictly follow the Spec-Kit Plus workflow (Spec > Clarify > Plan > Tasks > Implement).
- **AI-Native Authorship:** Claude Code subagents must be used for drafting, refining, validating, and maintaining book content.
- **Technical Accuracy:** All robotics, simulation, and AI details must be verifiable against current, official documentation (ROS 2, Gazebo, NVIDIA Isaac Sim 2023+, Unity and OpenAI, documentation).
- **Educational Clarity:** Content must be simple, clear, and example-based, focused on progressive learning for beginner to intermediate robotics learners.
- **AI-Augmented Learning:** The RAG chatbot must answer questions **ONLY** grounded in the book's text content.
- **Reusable Intelligence:** Prioritize the creation and documentation of reusable Subagents/Skills for recurring tasks (e.g., Docusaurus file headers, ROS 2 code snippets).


Key standards:
- **Written Format:** All content must be written in **MDX** for Docusaurus 3+.
- **Content Completeness:** Each chapter must include clear objectives, prerequisites, diagrams, step-by-step workflows, and runnable/validated code examples.
- **Book Structure:** Content must be logically organized into the **4 distinct course modules** (ROS 2, Digital Twin, Isaac Sim, VLA).
- **Source Verification:** All concepts must be traceable to primary research or official vendor documentation.
- **Reading Level & Tone:** Content must maintain a **Flesch-Kincaid Grade 9–12** reading level and a **technical, concise, and instructional** tone.
- **Chatbot Implementation:** The RAG chatbot must utilize a **free-tier architecture** and be built with modular code (e.g., API backend for processing).



Constraints:
- **Deliverable Minimums:** The final book must contain a minimum of **12 chapters** aligned with the 4 course modules(ROS 2 → Gazebo → Unity → Isaac → VLA), **30+ code examples**, **5 lab workflows**, and **1 capstone tutorial**.
- **Tooling:** Only official, supported tooling is allowed; no experimental APIs or hallucinated commands.
- **Personalization (Requirement Gate):** The final solution must support UI elements for personalization features (e.g., "Personalize Chapter" and "Translate to Urdu" buttons), whose implementation will be fully defined in the **Specify Phase**.
- **Platform Integrity:** Must be implemented in **Docusaurus** (Markdown + React/Typescript). Docusaurus builds must succeed without errors.
- **Deployment:** Final output must be deployable to **GitHub Pages**.
- **Integrated Code:** All code snippets must be testable and properly formatted for Docusaurus code block components.
- **Resource Efficiency:** The RAG and customization pipeline must utilize **minimal GPU usage** and **minimal embeddings** to adhere to the free-tier principle.
- **RAG Chatbot Requirement:** Chatbot must be functionally integrated, embedded in UI, capable of answering questions based *only* on the book's text content.

Success criteria:
- The entire project demonstrates adherence to the **Spec-Kit Plus workflow**.
- The Docusaurus site builds successfully and smoothly deploys to GitHub Pages.
- All core content modules are complete, technically accurate, and verifiable.
- The integrated RAG Chatbot is functional, polite, and answers questions correctly **only** using the book's content.
- Code (for RAG and personalization UI) is modular, secure, and adheres to modern standards.
- Clear **Architecture Decision Records (ADRs)** exist for all major architectural choices (e.g., database, API b
```

## Response snapshot

```text
New constitution version 1.0.0 has been created. The version bump is a major update as it's the initial creation and a significant update to the project's governance.

Files flagged for manual follow-up:
- TODO(RATIFICATION_DATE): Original adoption date unknown in .specify/memory/constitution.md. Please update this manually if known.

Suggested commit message:
```
docs: amend constitution to v1.0.0 (initial creation + governance update)
```
```

## Outcome

- ✅ Impact: Project constitution updated to version 1.0.0 with core principles, key standards, constraints, and governance defined.
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Proceed with feature specification or planning based on the updated constitution.
- 🧠 Reflection: Encountered issues with Bash execution of PowerShell script for PHR creation, fell back to agent-native method. Need to improve robustness of PHR creation script invocation.

## Evaluation notes (flywheel)

- Failure modes observed: Bash execution of PowerShell script failed due to path quoting/resolution issues.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate robust cross-platform script execution for PHR creation, possibly by adjusting shell invocation or direct Python execution if applicable.
