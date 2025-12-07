# Research & Decisions for Textbook Structure

**Date**: 2025-12-07
**Associated Plan**: [specs/main/plan.md](specs/main/plan.md)

This document summarizes the research conducted to resolve the "NEEDS CLARIFICATION" items identified in the implementation plan.

## 1. Docusaurus Glossary Search Component

### Research Task
Investigate the best way to implement a dedicated search component for the 100+ term glossary, as required by SC-005.

### Findings
- Docusaurus does not have a built-in, dedicated "glossary search" component out-of-the-box. The global search (Algolia) will index the glossary, but a dedicated, instant-search UI on the glossary page itself requires a custom implementation.
- The recommended approach is to create a custom React component. This component will:
  1.  Fetch glossary data from a static JSON file (e.g., `/static/glossary.json`).
  2.  Render a search input field.
  3.  Filter the glossary terms in real-time on the client-side based on user input.
  4.  Display the filtered list of terms and definitions.

### Decision
- A `glossary.json` file will be created to store the glossary terms and definitions.
- A new React component, `GlossarySearch.js`, will be developed in `src/components/`.
- This component will be embedded into the `docs/reference/glossary.md` file to provide the required search functionality.

---

## 2. Runnable Examples for VLA Models (GR00T)

### Research Task
Determine the best approach for creating runnable examples for advanced Vision-Language-Action (VLA) models like NVIDIA's GR00T within the specified learning environment (Isaac Sim, ROS 2).

### Findings
- Creating runnable examples for large foundation models like GR00T from scratch is highly complex and outside the scope of a typical textbook lab.
- NVIDIA provides official resources, including tutorials and GitHub repositories, with scripts for fine-tuning and running inference with their pre-trained models (e.g., GR00T N1.5).
- The primary workflow involves using Isaac Sim to generate synthetic data, fine-tune a model, and then roll out the learned policy in the simulator.
- The "Getting Started With Isaac for Healthcare" documentation and the `NVIDIA Isaac GR00T N1.5 GitHub repository` are key resources containing runnable code and Jupyter notebooks.

### Decision
- The textbook will not provide novel, self-contained runnable examples for VLAs.
- Instead, the VLA module (Weeks 11-13) will be structured as a guided walkthrough of the official NVIDIA tutorials.
- The chapters will provide the theoretical context, explain the steps, and link directly to the NVIDIA repositories and documentation. Students will be instructed to clone the NVIDIA repositories and run the provided scripts within the Isaac Sim environment.

---

## 3. Cloud-Native Setup (AWS/Azure)

### Research Task
Define the specific services and configurations for the cloud-native hardware setup path on AWS and Azure.

### Findings
- Both AWS and Azure offer official, pre-configured virtual machine images specifically for NVIDIA Isaac Sim.
- **AWS**:
  - **Image**: "NVIDIA Isaac Sim™ Development Workstation (Linux)" available in the AWS Marketplace.
  - **Recommended Instance**: `g6e.2xlarge` (features an NVIDIA L40S GPU).
  - **Connection**: NICE DCV client.
- **Azure**:
  - **Image**: "NVIDIA Isaac Sim Development Workstation VMI" available in the Azure Marketplace.
  - **Recommended Instance**: `Standard_NV36ads_A10_v5` (features an NVIDIA A10 GPU).
  - **Connection**: Remote Desktop for Windows or ThinLinc for Linux.
- A free NVIDIA developer account is a prerequisite for both platforms.

### Decision
- The `docs/setup/cloud-native.md` guide will provide step-by-step instructions for both AWS and Azure.
- The guide will link directly to the respective marketplace images.
- It will clearly state the recommended instance types and remind students of the NVIDIA developer account requirement and the fact that while the image is free, cloud infrastructure costs will apply.

---

## 4. Testing Strategy

### Research Task
Define a testing strategy for the Docusaurus site and the runnable Python code examples embedded within it.

### Findings
- A multi-faceted approach is required to ensure both documentation integrity and code correctness.
- **Docusaurus Site Testing**:
  - **Internal Links**: `docusaurus build` automatically checks for broken internal links.
  - **Markdown Quality**: Linters like `markdownlint-cli` and spell checkers like `cspell` can be used to maintain content quality.
  - **External Links**: Tools like `lychee` are needed to validate external URLs.
- **Python Code Example Testing**:
  - The most robust method is to extract the Python code blocks from the Markdown files.
  - A testing framework like `pytest` can then be used to execute each extracted code snippet.
  - Tests should assert that the code runs without raising exceptions. For examples that produce output, the `stdout` can be captured and compared against expected output.

### Decision
- A CI/CD pipeline (e.g., using GitHub Actions) will be established to automate testing.
- The CI pipeline will include the following steps:
  1.  Run `docusaurus build` to check for build errors and broken internal links.
  2.  Run `markdownlint-cli` and `cspell` on all documentation files.
  3.  Run an external link checker (`lychee`).
  4.  Execute a Python script that extracts all ` ```python ` code blocks from the `docs/` directory into temporary `.py` files.
  5.  Run `pytest` on the extracted Python files to ensure they are runnable and correct. The build will fail if any test fails.
