# Quickstart for Textbook Structure

**Date**: 2025-12-07
**Associated Plan**: [specs/main/plan.md](specs/main/plan.md)

This document provides a guide for developers and content creators to get started with the "Physical AI & Humanoid Robotics" textbook project structure.

## Prerequisites

- Node.js (v18 or later)
- Yarn (Docusaurus recommended package manager)
- A local clone of this repository.

## Getting Started

1.  **Install Dependencies**:
    Navigate to the root of the repository and install the necessary Node.js packages.
    ```bash
    yarn install
    ```

2.  **Start the Development Server**:
    To preview the Docusaurus site locally, run the following command:
    ```bash
    yarn start
    ```
    This will start a local development server and open up a browser window. Most changes are reflected live without having to restart the server. The site will be available at `http://localhost:3000`.

3.  **Build the Static Site**:
    To generate a static production build of the site, run:
    ```bash
    yarn build
    ```
    This command generates static content into the `build` directory and can be served using any static content hosting service. This is the command used for deployment to GitHub Pages.

## Adding and Editing Content

-   **Content Files**: All textbook content is located in the `docs/` directory. Files are written in Markdown (`.md` or `.mdx`).
-   **Structure**: The directory structure within `docs/` defines the URL structure of the site. For example, `docs/ros2/week4.md` will be available at `/docs/ros2/week4`.
-   **Metadata**: Every content file **must** include the YAML frontmatter defined in the `data-model.md` document. This metadata is critical for navigation, search, and maintaining a consistent structure.
-   **Navigation**: The sidebar navigation is controlled by the `sidebars.js` file in the root directory. To add a new file to the sidebar, you must add an entry to this file.

## API Contracts

This project is a Docusaurus website and does not expose a traditional REST or GraphQL API. The "contracts" for this project are:

1.  **The Data Model (`data-model.md`)**: The YAML frontmatter schema is the primary data contract for all content.
2.  **The Docusaurus Configuration (`docusaurus.config.js` and `sidebars.js`)**: These files define the structure, navigation, and behavior of the site.

The `specs/1-textbook-structure/contracts/` directory exists to conform to the planning template but is intentionally empty.
