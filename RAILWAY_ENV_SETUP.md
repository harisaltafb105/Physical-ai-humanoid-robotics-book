# Railway Environment Variables Setup

## Required Environment Variables for Railway

After pushing the code, Railway will automatically redeploy. You **MUST** set these environment variables in the Railway dashboard for the backend to work:

### 1. Navigate to Railway Dashboard
1. Go to your Railway project: `acceptable-alignment-production`
2. Click on your service
3. Go to **Variables** tab

### 2. Set These Environment Variables

```bash
# CORS Configuration (CRITICAL for Vercel)
CORS_ORIGINS=*

# OR for production security, use your Vercel domain:
# CORS_ORIGINS=https://physical-ai-humanoid-robotics-book-umber.vercel.app

# Application Settings
APP_ENV=production
DEBUG=False

# OpenAI Configuration
OPENAI_API_KEY=<your-openai-api-key>
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Vector Database
QDRANT_URL=<your-qdrant-cloud-url>
QDRANT_API_KEY=<your-qdrant-api-key>
QDRANT_COLLECTION_NAME=book_content

# Neon Serverless Postgres
DATABASE_URL=<your-postgresql-connection-string>

# Authentication
AUTH_SECRET=<your-random-secret-key>

# Session Management
SESSION_COOKIE_NAME=session_token
SESSION_MAX_AGE=604800

# Rate Limiting
RATE_LIMIT_REQUESTS_PER_MINUTE=10
```

### 3. Verify Deployment

After setting environment variables, Railway will automatically redeploy. Wait 2-3 minutes, then:

**Test the backend:**
```bash
curl https://acceptable-alignment-production.up.railway.app/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "openai": "connected",
    "qdrant": "connected",
    "database": "connected"
  }
}
```

**Test CORS:**
```bash
curl -i -H "Origin: https://physical-ai-humanoid-robotics-book-umber.vercel.app" \
  https://acceptable-alignment-production.up.railway.app/
```

Expected headers should include:
```
access-control-allow-origin: *
```

### 4. Vercel Environment Variable

Ensure Vercel has:
- **Variable:** `NEXT_PUBLIC_API_URL`
- **Value:** `https://acceptable-alignment-production.up.railway.app`

Then redeploy Vercel.

### 5. Troubleshooting

If Railway still returns 502:
1. Check Railway logs for errors
2. Verify all environment variables are set correctly
3. Ensure the Procfile and railway.json were detected (check Railway build logs)
4. Make sure Railway is connected to your GitHub repo and auto-deploys on push

## Summary of Changes Made

1. ✅ Fixed CORS configuration in `backend/src/config.py`
2. ✅ Added `Procfile` in root directory for Railway
3. ✅ Added `railway.json` with explicit build/deploy commands
4. ✅ Configured healthcheck endpoint
5. ✅ Committed and pushed to GitHub

## Next Steps

1. **Set environment variables in Railway** (see above)
2. **Wait for Railway to redeploy** (automatic, 2-3 minutes)
3. **Test backend health endpoint**
4. **Verify CORS headers**
5. **Test chatbot on Vercel**

Local development will continue to work as before with `NEXT_PUBLIC_API_URL=http://localhost:8000` in your local `.env` file.
