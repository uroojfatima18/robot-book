# Physical AI & Humanoid Robotics Book ok

A comprehensive educational platform for learning robotics, ROS 2, AI, and humanoid robot development.

## Project Structure

```
robot-book/
├── my-website/          # Frontend (Docusaurus)
│   ├── docs/            # Book chapters (formatted)
│   ├── src/             # Docusaurus components
│   ├── blog/            # Blog posts
│   └── static/          # Static assets
├── backend/             # Backend (Next.js API)
│   ├── src/             # API source code
│   │   ├── app/         # Next.js app router
│   │   ├── lib/         # Libraries (RAG, Gemini, DB)
│   │   └── components/  # UI components
│   └── scripts/         # Utility scripts
├── chapters/            # Source chapter content
├── specs/               # Feature specifications
├── history/             # Development history (PHRs)
├── archive/             # Archived review documents
└── .specify/            # Project configuration
```

## Quick Start

### Prerequisites
- Node.js 20+
- npm or yarn

### Installation

1. **Install frontend dependencies:**
   ```bash
   cd my-website
   npm install
   ```

2. **Install backend dependencies:**
   ```bash
   cd backend
   npm install
   ```

3. **Configure environment variables:**
   ```bash
   cd backend
   cp .env.example .env
   # Edit .env with your API keys
   ```

### Running the Application


**Terminal 1 - Backend:**
```bash
cd backend
uv run uvicorn main:app --reload --port 8001
```

**Terminal 2 - Frontend:**
```bash
cd my-website
npm run  start
```

### Accessing the Application
- **Frontend:** http://localhost:3005 (Docusaurus)
- **Backend API:** http://localhost:3000/api/chat/public

## Features

- **Interactive Book Content:** Learn ROS 2, digital twins, AI-powered robotics
- **AI Chatbot:** Ask questions about robotics topics
- **RAG-powered Answers:** Get accurate responses from verified book content


## Book Chapters

1. **ROS 2 Nervous System** - Robot communication fundamentals
2. **Digital Twin Simulation** - Virtual robot environments
3. **AI Robot Brain** - Machine learning for robotics
4. **Vision-Language-Action** - Task management and automation


## Tech Stack

### Frontend
- Docusaurus 3.x
- React 19
- TypeScript

### Backend
- Next.js 14 (App Router)
- Drizzle ORM
- Qdrant Vector DB
- OpenAI Agent SDK
- Openrouter API + model
- neon db

## License

All rights reserved.