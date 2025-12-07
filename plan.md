# Project Plan: Physical AI & Humanoid Robotics Textbook



## 1. Architectural Blueprint & Constraints



| Component | Technology / Requirement | Plan Alignment |

| :--- | :--- | :--- |

| **Frontend** | Docusaurus 3+ (MDX) + GitHub Pages | Structure/Metadata compliance (FR-004). |

| **Backend / AI** | FastAPI + Qdrant + Neon Postgres | RAG Chatbot grounded in text only (Constitution). |

| **Core Toolchain** | ROS 2 **Iron**, Isaac Sim **2023.1**, Ubuntu **22.04 LTS** (FR-010) | Strict technical accuracy. |

| **Module Structure** | 7 Distinct Content Areas (Foundations, Mods 1-4, Capstone, Mod 7 Evaluation) | 13-week progression adherence. |

| **CRITICAL FIX** | **FR-006 Assessment:** Replace "Gazebo" with "**Isaac Sim Assessment**". | Resolves conflict, aligns with Isaac stack. |



---



## 2. Implementation Phases & Task Breakdown



### ⚙️ Phase 1: Foundations & Structure (Weeks 1-3 Focus)

**Goal:** Establish Docusaurus base and critical setup documentation.



* **P1.1:** Init Docusaurus, configure **`docusaurus.config.js`** and **`sidebars.js`** (7 modules).

* **P1.2:** Create **Dashboard Homepage (`index.js`)** with module cards (FR-013).

* **P1.3:** Draft **3 Hardware Setup Guides** (`docs/setup/`) (FR-003).

* **P1.4:** Create Weeks 1-3 content, strictly applying **YAML Frontmatter Standard** (FR-004).



### 🧠 Phase 2: RAG Backend & Core Content (Weeks 4-10 Focus)

**Goal:** Develop ROS 2, Digital Twin, Isaac content, and RAG shell.



* **P2.1:** Develop **Modules 1-3** content (ROS 2, Digital Twin, Isaac), focusing on **runnable Python code** (FR-012).

* **P2.2:** Set up **FastAPI RAG Backend** shell with Qdrant and Neon connection logic.

* **P2.3:** Draft **Reference Files** (Glossary, Troubleshooting) (FR-007).



### 🚀 Phase 3: VLA, Capstone, and Final Integration (Weeks 11-13 Focus)

**Goal:** Complete VLA content, finalize Capstone, integrate RAG UI.



* **P3.1:** Develop **Module 4 (VLA)** content (LLM planning/VLM architectures).

* **P3.2:** Create **Capstone Project Guide** (FR-005) and **4 Assessment Guides** (ROS 2, **Isaac Sim**, Isaac, Capstone) with 3-level rubric (FR-006).

* **P3.3:** Develop **Module 7 (Evaluation)** content file.

* **P3.4:** Implement the **RAG Chatbot UI** and connect to the FastAPI backend.



### 🛡️ Phase 4: Quality Assurance and Deployment

**Goal:** Ensure technical compliance, security, and deployment readiness.



* **P4.1:** **RAG Security Audit:** Implement and test strict **input validation/sanitization** on the RAG API.

* **P4.2:** **Full Build & Test:** Verify successful Docusaurus build and RAG Chatbot grounding accuracy.

* **P4.3:** Configure GitHub Actions for deployment to GitHub Pages.