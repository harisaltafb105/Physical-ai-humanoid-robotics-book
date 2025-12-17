# Physical AI & Humanoid Robotics Book

An interactive educational book on Physical AI and Humanoid Robotics, covering ROS 2, Gazebo simulation, NVIDIA Isaac Sim, and Vision-Language-Action models.

## 🚀 Quick Start

### Prerequisites

- **Node.js**: Version 18 or higher
- **npm**: Version 9 or higher (comes with Node.js)
- **Git**: For version control

### Installation

1. Clone the repository:
```bash
git clone https://github.com/username/Physical-ai-human-robotics-book.git
cd Physical-ai-human-robotics-book
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

This will open your browser at `http://localhost:3000` with hot-reload enabled.

## 📚 Book Structure

The book is organized into 4 progressive modules:

- **Module 1: ROS 2 Fundamentals** (3 chapters, 36 min)
  - ROS 2 Basics & First Node
  - Topics & Services
  - URDF for Humanoids

- **Module 2: Gazebo Simulation** (2 chapters, 24 min)
  - Simulation Essentials
  - Sensors & ROS 2 Integration

- **Module 3: NVIDIA Isaac** (4 chapters, 48 min)
  - Isaac Sim Essentials
  - Synthetic Data & Perception
  - Visual SLAM
  - Navigation for Humanoids

- **Module 4: Vision-Language-Action** (3 chapters, 36 min)
  - VLA & Voice-to-Action
  - LLM Planning
  - Integration & Case Studies

**Total reading time**: ~3.4 hours

## 🛠️ Available Commands

- `npm start` - Start development server with hot-reload
- `npm run build` - Build production-ready static site
- `npm run serve` - Serve production build locally
- `npm run clear` - Clear Docusaurus cache
- `npm run lint:markdown` - Check markdown formatting
- `npm run lint:markdown:fix` - Auto-fix markdown issues

## 📖 Documentation

See [Quick Start Guide](./specs/001-physical-ai-robotics-book/quickstart.md) for detailed development environment setup.

## 🎯 Features

- ✅ Interactive Docusaurus-based book
- ✅ Dark/Light theme with Physical AI color palette
- ✅ Syntax highlighting for Python, Bash, YAML, XML
- ✅ Searchable content (Algolia DocSearch)
- ✅ Mobile-responsive design
- ✅ Automated deployment to GitHub Pages
- ✅ 80-100 runnable code examples
- ✅ Visual diagrams (SVG)

## 🔧 Development Workflow

1. **Create content**: Add markdown files in `docs/` directory
2. **Add diagrams**: Place SVG files in `static/img/`
3. **Update sidebar**: Edit `sidebars.js` to include new chapters
4. **Preview locally**: Run `npm start` to see changes in real-time
5. **Build and test**: Run `npm run build` to verify production build
6. **Push to GitHub**: Changes to `main` branch auto-deploy to GitHub Pages

## 📁 Project Structure

```
Physical-ai-human-robotics-book/
├── docs/                    # Markdown content (book chapters)
│   ├── intro.md            # Introduction
│   ├── quick-start.md      # Quick start guide
│   ├── module-1-ros2/      # Module 1 chapters
│   ├── module-2-gazebo/    # Module 2 chapters
│   ├── module-3-isaac/     # Module 3 chapters
│   ├── module-4-vla/       # Module 4 chapters
│   ├── hardware/           # Hardware guidance
│   └── glossary.md         # Glossary
├── static/                  # Static assets
│   └── img/                # Images and diagrams
├── src/                     # Custom components
│   └── css/                # Custom CSS
├── .github/workflows/       # GitHub Actions
├── docusaurus.config.js     # Site configuration
├── sidebars.js             # Sidebar navigation
└── package.json            # Dependencies
```

## 🌐 Deployment

### Frontend (Vercel/GitHub Pages)

#### Vercel Deployment

1. Connect your GitHub repository to Vercel
2. Set environment variable:
   - **Name**: `DOCUSAURUS_API_URL`
   - **Value**: `https://your-backend.railway.app` (your Railway backend URL)
3. Deploy

#### Automated GitHub Pages

Every push to the `main` branch automatically triggers:
1. Build process (`npm run build`)
2. Deployment to GitHub Pages

**Live site**: `https://username.github.io/Physical-ai-human-robotics-book/`

#### Manual Deployment

```bash
GIT_USER=username npm run deploy
```

### Backend (Railway)

The RAG chatbot backend is deployed separately on Railway:

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Set required environment variables on Railway:
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `QDRANT_URL` - Your Qdrant instance URL
   - `QDRANT_API_KEY` - Your Qdrant API key
   - `DATABASE_URL` - PostgreSQL connection string
   - `AUTH_SECRET` - Random 32-character secret
   - `CORS_ORIGINS` - Set to `*` to allow all origins (already configured in code)

3. Railway will automatically deploy when you push to the main branch

**Note**: The default CORS configuration allows all origins (`*`) for easy deployment. For production, you can restrict this by setting `CORS_ORIGINS` to specific domains like `https://yourdomain.vercel.app,https://yourdomain.com`

## 🧪 Testing

- **Markdown linting**: `npm run lint:markdown`
- **Link checking**: `npm run check:links` (after build)
- **Frontmatter validation**: `npm run validate:frontmatter`

## 📝 Contributing

See content creation guidelines in:
- [Module Schema Contract](./specs/001-physical-ai-robotics-book/contracts/module-schema.md)
- [Chapter Schema Contract](./specs/001-physical-ai-robotics-book/contracts/chapter-schema.md)
- [Code Example Schema Contract](./specs/001-physical-ai-robotics-book/contracts/code-example-schema.md)

## 📄 License

MIT License - see LICENSE file for details

## 🤝 Support

For issues and questions, please open a GitHub issue.

---

Built with [Docusaurus 3](https://docusaurus.io/) | Powered by Physical AI Education Team
