<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial ratification)
Modified Principles: N/A (initial creation)
Added Sections: All core principles, Documentation Standards, Development Workflow, Governance
Removed Sections: None
Templates Requiring Updates:
  ✅ plan-template.md - Constitution Check section ready to reference these principles
  ✅ spec-template.md - User scenarios align with Content-First principle
  ✅ tasks-template.md - Task structure supports Docusaurus deployment requirements
Follow-up TODOs: None
-->

# Physical AI & Human Robotics Book Constitution

## Core Principles

### I. Content-First Development

Content creation and accuracy MUST precede tooling and deployment concerns. All book chapters, sections, and educational material MUST be written with clear learning objectives and outcomes. Content MUST be technically accurate, peer-reviewable, and structured for progressive learning.

**Rationale**: Educational content quality directly impacts reader learning outcomes. Starting with content ensures the structure serves the material, not vice versa.

### II. Docusaurus Framework Adherence

All presentation layer MUST use Docusaurus framework conventions and best practices. Documentation MUST reference official Docusaurus documentation accessible via MCP Server Context 7. Configuration files (docusaurus.config.js, sidebars.js) MUST follow Docusaurus standards for maintainability and upgrade compatibility.

**Rationale**: Framework adherence ensures long-term maintainability, community support access, and predictable behavior across deployments.

### III. GitHub Pages Deployment Readiness

All code and configuration MUST be deployment-ready for GitHub Pages at any commit. Build artifacts MUST be reproducible and automated. Deployment configuration MUST include proper base URLs, routing, and asset paths for GitHub Pages hosting.

**Rationale**: Continuous deployment readiness enables rapid iteration and ensures production-grade code quality throughout development.

### IV. Progressive Content Structure

Book content MUST be organized hierarchically: chapters → sections → subsections. Navigation MUST support linear reading (prev/next) and non-linear exploration (sidebar, search). Content depth MUST increase progressively, with foundational concepts preceding advanced topics.

**Rationale**: Progressive structure supports diverse learning styles and enables readers to build knowledge systematically or explore specific topics as needed.

### V. Simplicity and Clarity

Content MUST prioritize clarity over cleverness. Code examples MUST be minimal, runnable, and directly illustrate the concept. Avoid unnecessary dependencies, complex build processes, or premature optimization. Each file MUST have a single, clear purpose.

**Rationale**: Educational materials succeed through clarity. Complexity obscures learning objectives and increases maintenance burden.

### VI. Version Control and Traceability

All changes MUST be committed with descriptive messages. Content revisions MUST be traceable to specific learning objectives or technical corrections. Documentation MUST include version history and last-updated dates for technical accuracy tracking.

**Rationale**: Version control enables collaborative editing, content auditing, and tracking of technical accuracy as the field evolves.

## Documentation Standards

### Content Quality Requirements

- **Technical Accuracy**: All technical claims MUST be verifiable and cite authoritative sources
- **Code Examples**: All code snippets MUST be tested and functional
- **Learning Objectives**: Each chapter MUST state clear learning objectives upfront
- **Progressive Difficulty**: Content difficulty MUST increase gradually with appropriate scaffolding
- **Accessibility**: Content MUST be readable at appropriate grade level with technical terms defined on first use

### Docusaurus-Specific Requirements

- **Metadata**: All markdown files MUST include frontmatter with id, title, and sidebar_position
- **MDX Features**: Use MDX components (admonitions, tabs, code blocks) appropriately to enhance learning
- **Images**: All images MUST be optimized for web and include descriptive alt text
- **Links**: Internal links MUST use Docusaurus-style references for portability
- **Search**: Content MUST be structured to support effective search functionality

## Development Workflow

### Content Development Cycle

1. **Research Phase**: Gather authoritative sources and outline learning objectives
2. **Draft Phase**: Write content in markdown following Docusaurus conventions
3. **Review Phase**: Verify technical accuracy and pedagogical effectiveness
4. **Integration Phase**: Add to Docusaurus project with proper navigation and metadata
5. **Build Phase**: Verify local build succeeds and content renders correctly
6. **Deployment Phase**: Deploy to GitHub Pages and verify production rendering

### Quality Gates

- **Pre-Commit**: Content MUST be spell-checked and markdown-linted
- **Pre-Build**: All internal links MUST resolve correctly
- **Pre-Deploy**: Build MUST succeed without warnings
- **Post-Deploy**: Live site MUST be manually verified for critical content

### Collaboration Standards

- **Content Ownership**: Each chapter MUST have a designated author/maintainer
- **Review Process**: Technical content MUST be reviewed by subject matter expert
- **Feedback Integration**: Reader feedback MUST be tracked and addressed in revision cycles
- **Attribution**: All external content (images, diagrams, code) MUST be properly attributed

## Governance

### Constitution Authority

This constitution governs all development decisions for the Physical AI & Human Robotics Book project. When conflicts arise between convenience and principles, principles MUST prevail with documented justification for any exceptions.

### Amendment Process

Constitution amendments MUST:
1. Be proposed with clear rationale and impact assessment
2. Undergo review by project stakeholders
3. Include migration plan for affected content and tooling
4. Update version number per semantic versioning rules

### Compliance and Review

- All pull requests MUST verify compliance with this constitution
- Complexity additions MUST be explicitly justified
- Quarterly reviews MUST assess adherence and identify improvement opportunities
- See CLAUDE.md for runtime agent development guidance

### Versioning Policy

Constitution version follows semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Backward-incompatible governance changes (e.g., removing content-first principle)
- **MINOR**: Additive changes (e.g., new documentation standard added)
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
