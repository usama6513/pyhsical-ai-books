---
id: 1
title: Initialize project constitution
stage: constitution
date: 2025-12-14
surface: agent
model: gemini-1.5-flash
feature: none
branch: N/A
user: N/A
command: /sp.constitution
labels: ["constitution", "initialization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

/sp.constitution /sp.constitution

Project: Comprehensive Physical AI Textbook - "Introduction to Artificial Intelligence: From Fundamentals to Practice"

Core Principles:
- Educational Excellence: Content must be pedagogically sound, progressing from basic to advanced concepts
- Practical Applicability: Every theoretical concept must include real-world examples and hands-on code demonstrations
- Accessibility: Complex AI concepts explained in clear, beginner-friendly language while maintaining technical accuracy
- Modern Relevance: Focus on current AI technologies (2024-2025), including LLMs, transformers, and modern ML frameworks
- Interactive Learning: Include exercises, quizzes, and project ideas at the end of each chapter
- Code-First Approach: Emphasis on practical implementation using Python, PyTorch, and TensorFlow

Key Standards:
- Content Structure: 
  * Modular chapter design compatible with Docusaurus
  * Each chapter: Introduction → Theory → Code Examples → Exercises → Summary
  * Consistent formatting using MDX components
  
- Technical Accuracy:
  * All code examples must be tested and executable
  * Mathematical formulas properly rendered using LaTeX
  * Diagrams and visualizations using Mermaid or React components
  * API references up-to-date with latest library versions

- Code Quality:
  * Python 3.10+ compatibility
  * PEP 8 style guidelines
  * Inline comments for complex logic
  * Full working examples (not pseudo-code)
  * Requirements.txt for each major section

- Documentation Standards:
  * Clear learning objectives at chapter start
  * Key takeaways at chapter end
  * Glossary of terms
  * Further reading suggestions with credible sources

- Source Credibility:
  * Primary sources: Research papers from arXiv, NeurIPS, ICML, ACL
  * Official documentation: PyTorch, TensorFlow, Hugging Face
  * Reputable blogs: Google AI Blog, OpenAI Blog, Anthropic Research
  * Minimum 70% academic/official sources

Constraints:
- Page Count: 250-350 pages (estimated in print format)
- Chapter Count: 12-15 chapters organized in 4 parts:
  * Part 1: AI Fundamentals (3-4 chapters)
  * Part 2: Machine Learning Deep Dive (4-5 chapters)
  * Part 3: Deep Learning & Neural Networks (3-4 chapters)
  * Part 4: Modern AI Applications (2-3 chapters)

- Technical Requirements:
  * Built using Docusaurus v3
  * Deployable to GitHub Pages
  * Mobile-responsive design
  * Dark/Light mode support
  * Search functionality
  * Interactive code playgrounds where possible

- Timeline: Hackathon duration (specify your timeline)
- File Structure: Organized per Spec-Kit Plus standards
- Version Control: Git with meaningful commits

Success Criteria:
- ✅ Complete book structure with all chapters outlined
- ✅ At least 80% of code examples are runnable
- ✅ Successfully deployed to GitHub Pages
- ✅ Mobile-responsive and accessible (WCAG AA)
- ✅ Navigation works seamlessly
- ✅ All images and diagrams render correctly
- ✅ Search functionality operational
- ✅ Zero broken links or missing resources
- ✅ Professional presentation suitable for physical printing
- ✅ Passes technical review for accuracy
- ✅ Includes downloadable code repository
- ✅ PDF export option available

Target Audience:
- Primary: Computer Science undergraduates, bootcamp students
- Secondary: Self-learners with basic programming knowledge
- Prerequisite knowledge: Python basics, basic mathematics (algebra, calculus)

Special Features:
- Interactive elements using React components in MDX
- Syntax-highlighted code blocks
- Collapsible sections for advanced topics
- Progress tracking (if implemented)
- Downloadable Jupyter notebooks
- Video embed support for visual explanations

Quality Assurance:
- Peer review by at least 2 technical reviewers
- User testing with target audience sample
- Automated link checking
- Cross-browser testing
- Print layout verification

## Response snapshot

New constitution version 0.1.0 has been set based on your input.

**Manual Follow-up Required:**
The following templates require manual review and potential updates to align with the new constitution:
- `.specify/templates/plan-template.md`
- `.specify/templates/spec-template.md`
- `.specify/templates/tasks-template.md`
- `.specify/templates/commands/*.md`

**Suggested Commit Message:**
```
docs: initialize constitution to v0.1.0 (detailed principles + standards)
```

## Outcome

- ✅ Impact: Constitution file created and updated successfully.
- 🧪 Tests: None
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Review and update dependent templates.
- 🧠 Reflection: PHR creation script failed in powershell, used agent-native method.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` is a bash script and failed in a powershell environment.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Improve cross-platform compatibility of PHR creation scripts or provide direct powershell equivalent.
