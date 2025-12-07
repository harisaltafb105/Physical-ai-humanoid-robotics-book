# Quick Start: Development Environment Setup

**Feature**: Physical AI & Humanoid Robotics Book
**Phase**: 1 - Design & Contracts
**Date**: 2025-12-06

## Purpose

This guide helps developers set up a local Docusaurus development environment to build, preview, and contribute to the Physical AI & Humanoid Robotics book.

**Target Audience**: Content authors, technical reviewers, and contributors working on the book.

## Prerequisites

- **Node.js**: Version 18 or higher
- **npm**: Version 9 or higher (comes with Node.js)
- **Git**: For version control
- **Text Editor**: VS Code recommended (with Markdown linting extensions)
- **Operating System**: Windows, macOS, or Linux

## Installation Steps

### 1. Install Node.js

**Check if already installed**:
```bash
node --version
npm --version
```

**If not installed**:
- **macOS**: `brew install node`
- **Windows**: Download from [nodejs.org](https://nodejs.org/)
- **Linux (Ubuntu/Debian)**:
  ```bash
  curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
  sudo apt-get install -y nodejs
  ```

### 2. Clone the Repository

```bash
git clone https://github.com/<username>/Physical-ai-human-robotics-book.git
cd Physical-ai-human-robotics-book
```

Replace `<username>` with actual GitHub username or organization.

### 3. Install Dependencies

```bash
npm install
```

This installs Docusaurus and all required plugins (defined in `package.json`).

**Expected Output**:
```
added 1234 packages, and audited 1235 packages in 45s
found 0 vulnerabilities
```

### 4. Start Local Development Server

```bash
npm start
```

**What This Does**:
- Builds the documentation site
- Starts a local web server at `http://localhost:3000`
- Opens your default browser automatically
- Watches for file changes and hot-reloads

**Expected Output**:
```
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

**To Stop**: Press `Ctrl+C` in the terminal.

## Development Workflow

### Directory Structure

```
Physical-ai-human-robotics-book/
├── docs/                    # Markdown content (your book chapters)
│   ├── intro.md            # Homepage
│   ├── quick-start.md      # Quick start guide
│   ├── module-1-ros2/      # Module 1 chapters
│   ├── module-2-digital-twin/
│   ├── module-3-nvidia-isaac/
│   ├── module-4-vla/
│   ├── hardware/           # Hardware guidance
│   └── glossary.md         # Glossary
├── static/                  # Static assets
│   └── img/                # Images and diagrams
├── docusaurus.config.js    # Site configuration
├── sidebars.js             # Sidebar navigation
└── package.json            # Node.js dependencies
```

### Adding a New Chapter

1. **Create Markdown File**:
   ```bash
   cd docs/module-1-ros2
   touch 06-new-chapter.md
   ```

2. **Add Frontmatter** (YAML metadata at top of file):
   ```yaml
   ---
   id: new-chapter-id
   title: New Chapter Title
   sidebar_position: 6
   last_updated: 2025-12-06
   ---
   ```

3. **Write Content**: Follow the [Chapter Schema Contract](./contracts/chapter-schema.md)

4. **Preview**: Save the file, and the local server will auto-reload

5. **Update Sidebar** (if needed): Edit `sidebars.js` to add the chapter to navigation

### Adding an Image

1. **Add Image File**:
   ```bash
   cp path/to/diagram.svg static/img/my-diagram.svg
   ```

2. **Reference in Markdown**:
   ```markdown
   ![Descriptive alt text](../../static/img/my-diagram.svg)
   ```

3. **Verify**: Check that the image displays correctly in browser

### Code Examples

Follow the [Code Example Schema Contract](./contracts/code-example-schema.md):

````markdown
```python
#!/usr/bin/env python3
"""
Description of what this code does.
Part of Module N, Chapter M: Chapter Title
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
# ... code here

# Expected output:
# [INFO] Example output
```
````

## Building for Production

### Local Build

Test the production build locally:

```bash
npm run build
```

**Output**: Static site generated in `build/` directory

**Expected Output**:
```
[SUCCESS] Generated static files in build/
[SUCCESS] Use `npm run serve` to test your build locally
```

### Serve Production Build

```bash
npm run serve
```

Opens production build at `http://localhost:3000/` (without hot reload).

## Validation

### Markdown Linting

Check for markdown formatting issues:

```bash
npm run lint:markdown
```

**Fixes common issues**:
```bash
npm run lint:markdown:fix
```

### Link Checking

Verify all internal links are valid:

```bash
npm run check:links
```

This runs only after `npm run build` (checks the built site).

### Frontmatter Validation

Validate chapter frontmatter (custom script):

```bash
npm run validate:frontmatter
```

Checks:
- All required fields present (id, title, sidebar_position, last_updated)
- Unique IDs and sidebar positions
- Valid date formats

## Troubleshooting

### Issue: "Cannot find module 'docusaurus'"

**Solution**: Run `npm install` to install dependencies

### Issue: Port 3000 already in use

**Solution**: Stop other dev servers, or use different port:
```bash
npm start -- --port 3001
```

### Issue: Changes not reflecting in browser

**Solution**:
1. Hard refresh: `Ctrl+Shift+R` (Windows/Linux) or `Cmd+Shift+R` (Mac)
2. Clear browser cache
3. Restart dev server (`Ctrl+C` then `npm start`)

### Issue: Build fails with "Broken links found"

**Solution**:
1. Check error message for specific broken link
2. Fix the link in markdown file
3. Re-run `npm run build`

### Issue: Hot reload not working

**Solution**: Edit `docusaurus.config.js` and verify `livereload: true` in dev server config

## Best Practices

### 1. Write in Small Commits

```bash
git add docs/module-1-ros2/02-first-ros2-node.md
git commit -m "docs: add first ROS 2 node tutorial"
```

### 2. Preview Before Pushing

Always run `npm run build` to catch errors before pushing to GitHub.

### 3. Follow Content Contracts

Adhere to:
- [Module Schema Contract](./contracts/module-schema.md)
- [Chapter Schema Contract](./contracts/chapter-schema.md)
- [Code Example Schema Contract](./contracts/code-example-schema.md)

### 4. Keep Reading Time in Check

Each chapter should be ~12 minutes (1,500-2,000 words). Check word count:

```bash
wc -w docs/module-1-ros2/02-first-ros2-node.md
```

### 5. Test Code Examples

Before publishing, copy-paste code examples into a terminal to verify they work.

## Deployment

### Automated (GitHub Actions)

When you push to the `main` branch, GitHub Actions automatically:
1. Runs `npm ci` (clean install)
2. Runs `npm run build`
3. Deploys `build/` directory to GitHub Pages

**Workflow File**: `.github/workflows/deploy.yml`

**Live Site**: `https://<username>.github.io/Physical-ai-human-robotics-book/`

### Manual Deployment (if needed)

```bash
GIT_USER=<username> npm run deploy
```

This builds and pushes to `gh-pages` branch manually.

## VS Code Setup (Recommended)

### Extensions

Install these VS Code extensions:
- **Markdown All in One** (Yu Zhang): Shortcuts, preview, TOC generation
- **markdownlint** (David Anson): Markdown linting
- **Code Spell Checker** (Street Side Software): Catch typos
- **Markdown Preview Enhanced** (Yiyi Wang): Advanced preview features

### Workspace Settings

Create `.vscode/settings.json`:

```json
{
  "editor.formatOnSave": true,
  "editor.codeActionsOnSave": {
    "source.fixAll.markdownlint": true
  },
  "[markdown]": {
    "editor.wordWrap": "on",
    "editor.rulers": [80]
  },
  "markdownlint.config": {
    "MD013": false,
    "MD033": false
  }
}
```

This auto-fixes markdown issues on save.

## Additional Resources

- **Docusaurus Documentation**: https://docusaurus.io/docs
- **Markdown Guide**: https://www.markdownguide.org/
- **MDX (Markdown + JSX)**: https://mdxjs.com/

## Summary

**Quick Commands**:
- `npm install` - Install dependencies (first time)
- `npm start` - Start dev server (hot reload)
- `npm run build` - Production build
- `npm run serve` - Test production build
- `npm run lint:markdown` - Check markdown formatting
- `npm run check:links` - Validate internal links

**Next Steps**: Start writing content! Begin with the introduction or dive into Module 1.
