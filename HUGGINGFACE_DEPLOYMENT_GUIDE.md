# Hugging Face Spaces - Backend Deployment (Monorepo)

## Prerequisites

1. Hugging Face account (free at https://huggingface.co)
2. GitHub repo already set up with frontend & backend
3. Docker installed (for local testing)

## Your Project Structure

```
robot-book/ (GitHub repo root)
├── backend/
│   ├── main.py
│   ├── run_backend.py
│   ├── requirements.txt
│   ├── app/
│   └── ...
├── my-website/
│   ├── (frontend - already deployed)
│   └── ...
├── .env.local
└── ...
```

## Step-by-Step Deployment

### Step 1: Create Dockerfile at Repository Root

Create a `Dockerfile` in the root of your `robot-book` GitHub repo:

```dockerfile
FROM python:3.12-slim as builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy backend requirements
COPY backend/requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

# Runtime stage
FROM python:3.12-slim

WORKDIR /app

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy Python dependencies from builder
COPY --from=builder /root/.local /root/.local

# Copy backend code from backend folder
COPY backend/ /app/

# Set environment variables
ENV PATH=/root/.local/bin:$PATH \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PORT=7860

# Expose port (Hugging Face Spaces standard)
EXPOSE 7860

# Create start script
RUN echo '#!/bin/bash\npython -c "import uvicorn; from main import app; uvicorn.run(app, host=\"0.0.0.0\", port=7860, log_level=\"info\")"' \
    > /app/start.sh && chmod +x /app/start.sh

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD curl -f http://localhost:7860/api/health || exit 1

# Run the application
CMD ["/app/start.sh"]
```

Add to GitHub:
```bash
git add Dockerfile
git commit -m "Add Dockerfile for HF Spaces backend deployment"
git push origin main
```

### Step 2: Create Hugging Face Space

1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Configure:
   - **Space name**: `robot-book-backend`
   - **License**: MIT
   - **Space SDK**: Docker
   - **Visibility**: Private (recommended) or Public
4. Click **"Create Space"**

### Step 3: Connect Your GitHub Repository

In your newly created Hugging Face Space:

1. Click **Settings** tab
2. Scroll to **Linked Repositories**
3. Click **"Link to repository"**
4. Choose **GitHub** as source
5. Select your `username/robot-book` repo
6. Click **"Link"**

The space will automatically sync with GitHub and build.

**Alternative (Manual Push):**
```bash
cd robot-book

# Add HF remote
git remote add huggingface https://huggingface.co/spaces/YOUR_USERNAME/robot-book-backend

# Push to HF
git push huggingface main
```

### Step 4: Configure Environment Variables

In your Hugging Face Space:

1. Click **Settings**
2. Go to **Repository secrets**
3. Add these secrets:
   - `OPENAI_API_KEY` = Your OpenAI API key
   - `OPENROUTER_API_KEY` = Your OpenRouter API key
   - `GOOGLE_API_KEY` = Your Google API key
   - `DATABASE_URL` = (Optional) PostgreSQL URL
   - `QDRANT_URL` = (Optional) Vector DB URL

⚠️ **Never commit `.env` files to GitHub!**

### Step 5: Deployment Complete

Your space will:
1. Automatically build the Docker image
2. Deploy within 5-10 minutes
3. Be available at: `https://YOUR_USERNAME-robot-book-backend.hf.space`

Check the **Logs** tab to monitor the build.

## API Endpoints

Once deployed:

- **Base URL**: `https://YOUR_USERNAME-robot-book-backend.hf.space`
- **Health**: `GET https://YOUR_USERNAME-robot-book-backend.hf.space/api/health`
- **Chat**: `POST https://YOUR_USERNAME-robot-book-backend.hf.space/api/ask`

## Update Frontend URL

In `my-website/.env.local` or `my-website/.env.production`:

```env
NEXT_PUBLIC_API_BASE_URL=https://YOUR_USERNAME-robot-book-backend.hf.space/api
```

Then redeploy your frontend with the new URL.

## Local Testing Before Pushing

Test the Docker build locally:

```bash
# Build from root directory
docker build -t robot-book-backend:test .

# Run on port 7860 (matching HF Spaces)
docker run -d \
  -p 7860:7860 \
  -e OPENAI_API_KEY=test_key \
  -e OPENROUTER_API_KEY=test_key \
  -e GOOGLE_API_KEY=test_key \
  robot-book-backend:test

# Test health endpoint
curl http://localhost:7860/api/health

# View logs
docker logs -f container_id
```

## Updating Deployment

### Automatic Updates (Recommended)

Just push changes to GitHub:
```bash
git add .
git commit -m "Update backend code"
git push origin main
```

Hugging Face automatically detects changes and rebuilds.

### Manual Restart

In your Space:
1. Go to **Settings**
2. Click **Restart** → **Hard restart**
3. Docker image rebuilds and redeploys

## Monitoring

### View Logs
- Click the **Logs** tab in your Space
- Real-time container output is displayed

### Keep Space Active
Free tier spaces sleep after 2 months of inactivity:

```bash
# Add a periodic check to keep it awake (run externally)
curl https://YOUR_USERNAME-robot-book-backend.hf.space/
```

Upgrade to paid tier to prevent auto-sleep.

## Important Notes

### Port Configuration
- Dockerfile uses port **7860** (Hugging Face standard)
- Production ready ✅

### API Keys Security
- ✅ Use Hugging Face Repository Secrets (NOT hardcoded)
- ✅ Secrets are injected as environment variables at runtime
- ✅ Never commit `.env` files
- ✅ Use `.gitignore` to exclude sensitive files

### Resource Limits (Free Tier)
- CPU: 2 cores
- RAM: 16GB
- Storage: 50GB
- Auto-sleeps: After 2 months of inactivity

Upgrade to paid tier ($5/mo) for always-on, better resources.

## Troubleshooting

### Docker Build Fails
1. Check **Logs** tab in your Space
2. Ensure `backend/requirements.txt` exists
3. Verify Python 3.12 compatibility

### Port Binding Error
- Already handled in Dockerfile
- Dockerfile uses port 7860 (HF standard)

### Environment Variables Not Working
- Verify secret names in Repository Secrets
- Restart the Space after adding secrets
- Names must match exactly

### Timeout Issues
- HF has ~5 minute request timeout
- Use async operations for long tasks
- Consider background job queues

## Monorepo Workflow

✅ **This setup works because:**
- Dockerfile is at repo root
- Uses `COPY backend/` to copy only backend files
- Frontend in `my-website/` is not included in Docker image
- Both can be deployed separately from same GitHub repo

✅ **Git workflow:**
```bash
# One repo push to GitHub
git push origin main

# Frontend deploys via its own CI/CD
# Backend deploys via HF Space GitHub link (automatic)
```

## Cost Comparison

| Aspect | Free Tier | Paid ($5/mo) |
|--------|-----------|------|
| CPU | 2 cores | More |
| RAM | 16GB | More |
| Storage | 50GB | More |
| Always-on | ❌ (sleeps) | ✅ |
| Duration | 2 months inactive | Unlimited |

Recommended: Use paid tier for production backends.

## Support

- Hugging Face Docs: https://huggingface.co/docs/hub/spaces
- Docker Docs: https://docs.docker.com/
- Your Repo: https://github.com/YOUR_USERNAME/robot-book
