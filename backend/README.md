# Robot Book Backend

FastAPI backend for the Robot Book RAG Chatbot. Features streaming responses, vector search via Qdrant, and authentication via Better Auth.

## Quick Start

### Local Development

1. **Clone and setup:**
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

2. **Configure environment:**
```bash
cp .env.example .env
# Edit .env with your API keys
```

3. **Run the server:**
```bash
python run_backend.py
# Server runs on http://localhost:8001
```

### API Endpoints

- **GET `/`** - Welcome message
- **GET `/api/health`** - Health check
- **POST `/api/ask`** - Chat endpoint (streaming)
- **GET `/docs`** - FastAPI Swagger documentation

## Environment Variables

### Required

| Variable | Description | Example |
|----------|-------------|---------|
| `OPENROUTER_API_KEY` | OpenRouter API key | `sk-or-v1-...` |
| `QDRANT_URL` | Qdrant instance URL | `https://your-instance.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `eyJ...` |
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@host/db` |
| `EMAIL_FROM` | Sender email (for Resend) | `noreply@example.com` |

### Optional

| Variable | Description | Default |
|----------|-------------|---------|
| `FRONTEND_URL` | Frontend URL for CORS | `http://localhost:3000` |
| `COLLECTION_NAME` | Qdrant collection name | `book_chunks` |
| `DOCS_PATH` | Book docs path | `../my-website/docs` |
| `BETTER_AUTH_URL` | Auth service URL | `http://localhost:3001` |
| `BETTER_AUTH_SECRET` | Auth secret key | - |
| `OPENAI_API_KEY` | OpenAI API key (embeddings) | Optional fallback |
| `RESEND_API_KEY` | Resend email API key | Optional |

## Deployment on Hugging Face Spaces

### Setup Instructions

1. **Create a Space:**
   - Go to https://huggingface.co/spaces
   - Create new Space with Docker SDK
   - Name: `robot-book-backend`

2. **Clone the Space repository:**
```bash
git clone https://huggingface.co/spaces/YOUR_USERNAME/robot-book-backend
cd robot-book-backend
```

3. **Copy files:**
```bash
# Copy Dockerfile from main repo
cp ../robot-book/Dockerfile .

# Copy backend files
cp -r ../robot-book/backend/* .
```

4. **Commit and push:**
```bash
git add .
git commit -m "Add backend files"
git push
```

5. **Configure secrets:**
   - In Space Settings → Repository secrets
   - Add all required environment variables (see table above)
   - DO NOT use `.env` files - use Space secrets only

### Environment Variables for HF Spaces

Add these in **Settings → Repository secrets**:

```
OPENROUTER_API_KEY=sk-or-v1-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
EMAIL_FROM=your-email@example.com
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=...
OPENAI_API_KEY=...
RESEND_API_KEY=...
FRONTEND_URL=https://your-frontend-url.example.com
```

### Your Deployed API URL

After deployment (5-10 minutes):
```
https://USERNAME-robot-book-backend.hf.space/api
```

## Architecture

```
backend/
├── main.py              # FastAPI app & routes
├── run_backend.py       # Server startup
├── app/
│   ├── models.py        # Pydantic schemas
│   ├── services/        # Business logic
│   └── ...
├── requirements.txt     # Python dependencies
├── Dockerfile          # Docker configuration
└── .env.example        # Environment template
```

## Key Features

- **Streaming Responses**: SSE (Server-Sent Events) for real-time chat
- **Vector Search**: Qdrant for semantic search over book content
- **Authentication**: Better Auth integration
- **Email Support**: Resend for transactional emails
- **Database**: PostgreSQL (Neon) for metadata storage
- **FastAPI**: Fast, async Python web framework
- **CORS**: Configured for frontend integration

## Local Testing

```bash
# Health check
curl http://localhost:8001/api/health

# Ask a question
curl -X POST http://localhost:8001/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS?"}'

# View docs
open http://localhost:8001/docs
```

## Docker Build

```bash
# Build locally
docker build -t robot-book-backend .

# Run locally
docker run -d \
  -p 7860:7860 \
  -e OPENROUTER_API_KEY=your_key \
  -e QDRANT_URL=your_url \
  -e QDRANT_API_KEY=your_key \
  -e DATABASE_URL=your_db_url \
  robot-book-backend
```

## Dependencies

- **fastapi** - Web framework
- **uvicorn** - ASGI server
- **qdrant-client** - Vector DB client
- **asyncpg** - PostgreSQL async driver
- **openai** - OpenAI API client
- **python-dotenv** - Environment configuration
- **pydantic** - Data validation

## Troubleshooting

### Import Errors
```bash
# Reinstall dependencies
pip install -r requirements.txt --force-reinstall
```

### Database Connection Issues
- Verify `DATABASE_URL` is correct
- Check PostgreSQL is accessible
- Ensure SSL mode is set properly (`?sslmode=require`)

### Qdrant Connection Issues
- Verify `QDRANT_URL` is accessible
- Check `QDRANT_API_KEY` is valid
- Ensure collection exists in Qdrant

### API Not Responding
```bash
# Check health endpoint
curl http://localhost:8001/api/health

# View logs
docker logs container_id
```

## Security Notes

⚠️ **Never commit `.env` files to Git**
- Always use environment variables
- On HF Spaces, use Repository secrets
- Rotate API keys regularly
- Use minimum required permissions

## Contributing

1. Create a feature branch
2. Make changes
3. Test locally
4. Push and create PR

## Support

- Issues: GitHub Issues
- Docs: `/docs` endpoint (Swagger UI)
- HF Spaces: https://huggingface.co/spaces/YOUR_USERNAME/robot-book-backend

---

**Last Updated**: 2026-04-05
**Python Version**: 3.12+
**License**: MIT
