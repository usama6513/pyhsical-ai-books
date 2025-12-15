<!--
Sync Impact Report:
Version change: 0.1.0 -> 1.0.1
List of modified principles:
  - Educational Excellence -> Design Philosophy: The Cyber-Physical Look
  - Practical Applicability -> Component Requirements: Landing Page
  - Accessibility -> Component Requirements: Global Styling
  - Modern Relevance -> Configuration: docusaurus.config.js Navbar
  - Interactive Learning -> Immediate Actions
  - Code-First Approach (Removed)
Added sections: None
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .specify/templates/commands/sp.adr.toml (⚠ pending)
  - .specify/templates/commands/sp.analyze.toml (⚠ pending)
  - .specify/templates/commands/sp.checklist.toml (⚠ pending)
  - .specify/templates/commands/sp.clarify.toml (⚠ pending)
  - .specify/templates/commands/sp.constitution.toml (⚠ pending)
  - .specify/templates/commands/sp.git.commit_pr.toml (⚠ pending)
  - .specify/templates/commands/sp.implement.toml (⚠ pending)
  - .specify/templates/commands/sp.phr.toml (⚠ pending)
  - .specify/templates/commands/sp.plan.toml (⚠ pending)
  - .specify/templates/commands/sp.specify.toml (⚠ pending)
  - .specify/templates/commands/sp.tasks.toml (⚠ pending)
Follow-up TODOs:
- Key Standards, Constraints, Governance sections need to be fully defined.
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Design Philosophy: The Cyber-Physical Look
The UI must feel like a futuristic interface for controlling robots. Primary Vibe: Modern, Clean, Dark-Mode First. Color Palette: Primary Color: Electric Blue / Cyan (representing AI/Electricity) instead of default Green. Background: Deep Slate/Black (for high contrast). Accents: Neon Purple (for "AI Brain" concepts).

### Component Requirements: Landing Page
Hero Section: Must be bold and centered. Headline: "Build the Body. Code the Brain." Sub-headline: "The World's First Spec-Driven Course on Physical AI & Humanoid Robotics. Written by Sharmeen Fatima." Buttons: Two big buttons: "Start Learning (Module 1)" and "View Syllabus". Feature Cards: Below the hero, display 3 cards: "🤖 Embodied Intelligence", "🧠 Spec-Driven Development", "⚡ Sim-to-Real Transfer".

### Component Requirements: Global Styling
Fonts: Use a clean sans-serif font (Inter or Roboto) for readability. Admonitions: Custom style for :::tip, :::warning, etc., to look like "System Alerts" (e.g., rounded corners, slight glow). Navbar: Sticky, with a glass-morphism effect (blur background).

### Configuration: docusaurus.config.js Navbar
Navbar Items: Left: Logo + "Physical AI Textbook". Right: "Modules" (Dropdown), "GitHub" (linking to https://github.com/Sharmeen-Fatima), "Panaversity". Footer: Clean footer with copyright and social links. Include: "© 2025 Sharmeen Fatima. All rights reserved." followed by links to GitHub and LinkTree. Style the links as icons if possible (e.g., GitHub icon for the GitHub link).

### Immediate Actions
IMMEDIATELY update docusaurus.config.js to change the primaryColor config. IMMEDIATELY rewrite src/css/custom.css with these new colors and styles. IMMEDIATELY rewrite src/pages/index.js to create the new Hero section.

## Key Standards
To be defined.

## Constraints
To be defined.

## Governance
Amendments to this constitution require a documented proposal, review by at least two project leads, and majority approval. Versioning follows semantic versioning rules (MAJOR.MINOR.PATCH).

**Version**: 1.0.1 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14