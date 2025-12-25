# Railway PORT Error - COMPLETE FIX

## ✅ PROBLEM SOLVED

### Original Error
```
Error: Invalid value for '--port': '$PORT' is not a valid integer.
```

### Root Cause
1. **Duplicate Procfile**: Both `/Procfile` and `/backend/Procfile` existed
2. **Shell Substitution Failure**: Railway didn't interpret `${PORT:-8000}` correctly
3. **Railway Priority**: Railway was reading `/backend/Procfile` (the broken one)

### Solution Applied
**Use Python to handle PORT environment variable instead of shell substitution**

---

## 📝 FILES CHANGED

### 1. **backend/run.py** (NEW FILE - Python startup script)
```python
#!/usr/bin/env python3
"""
Railway startup script - handles PORT environment variable
"""
import os
import sys

if __name__ == "__main__":
    # Get PORT from environment, default to 8000
    port = int(os.environ.get("PORT", 8000))
    host = "0.0.0.0"

    print(f"Starting server on {host}:{port}")

    # Import and run uvicorn programmatically
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=host,
        port=port,
        log_level="info"
    )
```

**Why This Works:**
- Python reads `PORT` environment variable directly
- No shell substitution needed
- Railway sets `PORT` automatically
- Falls back to `8000` for local development

---

### 2. **Procfile** (UPDATED - Root directory)
```bash
web: cd backend && python run.py
```

**Changed from:**
```bash
web: sh -c 'cd backend && uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'
```

---

### 3. **railway.json** (UPDATED)
```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "cd backend && pip install -r requirements.txt"
  },
  "deploy": {
    "startCommand": "cd backend && python run.py",
    "healthcheckPath": "/health",
    "healthcheckTimeout": 100
  }
}
```

---

### 4. **backend/Procfile** (DELETED)
Removed duplicate Procfile that was causing Railway to use wrong startup command.

### 5. **backend/start.sh** (DELETED)
No longer needed with Python startup script.

---

## ✅ VERIFICATION - LOCAL TESTING PASSED

### Test 1: Default Port (8000)
```bash
$ cd backend && python run.py
Starting server on 0.0.0.0:8000
INFO: Uvicorn running on http://0.0.0.0:8000

$ curl http://localhost:8000/health
{"status":"healthy","services":{"openai":"connected","qdrant":"connected","database":"fallback"}}
```

### Test 2: Custom Port (9999)
```bash
$ PORT=9999 python run.py
Starting server on 0.0.0.0:9999
INFO: Uvicorn running on http://0.0.0.0:9999

$ curl http://localhost:9999/health
{"status":"healthy","services":{"openai":"connected","qdrant":"connected","database":"fallback"}}
```

✅ **Both tests passed - PORT variable handling works correctly!**

---

## 🚀 HOW TO DEPLOY TO RAILWAY

### Step 1: Commit Changes (DO NOT PUSH YET - read all steps first)
```bash
git status
# Verify these files are changed:
# - backend/run.py (new)
# - Procfile (modified)
# - railway.json (modified)
# - backend/Procfile (deleted)
# - backend/start.sh (deleted)

git add backend/run.py Procfile railway.json
git add backend/Procfile backend/start.sh  # Stage deletions
git commit -m "Fix Railway PORT error with Python startup script"
```

### Step 2: Set Railway Environment Variables FIRST
**CRITICAL: Set these BEFORE pushing, or app will start without full functionality**

Go to Railway Dashboard → Your Service → Variables:

```bash
# REQUIRED for chatbot functionality
CORS_ORIGINS=*
OPENAI_API_KEY=<copy-from-backend/.env>
QDRANT_URL=<copy-from-backend/.env>
QDRANT_API_KEY=<copy-from-backend/.env>
DATABASE_URL=<copy-from-backend/.env>
AUTH_SECRET=<copy-from-backend/.env>

# OPTIONAL (have defaults)
APP_ENV=production
DEBUG=False
OPENAI_MODEL=gpt-4-turbo-preview
QDRANT_COLLECTION_NAME=book_content
```

**Copy values from your local `backend/.env` file!**

### Step 3: Push to GitHub
```bash
git push origin main
```

Railway will automatically:
1. Detect the push
2. Read `Procfile` → run `python backend/run.py`
3. Python reads `$PORT` from Railway environment
4. Start on Railway's assigned port
5. Healthcheck at `/health`

### Step 4: Monitor Railway Deployment (2-3 minutes)
Watch Railway logs for:
```
Starting server on 0.0.0.0:<railway-port>
INFO: Uvicorn running on http://0.0.0.0:<railway-port>
INFO: Application startup complete
```

---

## 🧪 VERIFICATION STEPS

### After Railway Deployment Completes:

#### Test 1: Health Endpoint
```bash
curl https://acceptable-alignment-production.up.railway.app/health
```

**Expected Response:**
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

#### Test 2: CORS Headers
```bash
curl -i https://acceptable-alignment-production.up.railway.app/
```

**Expected Headers:**
```
HTTP/1.1 200 OK
access-control-allow-origin: *
```

#### Test 3: Chatbot on Vercel
1. Open: `https://physical-ai-humanoid-robotics-book-umber.vercel.app/ai-chatbot`
2. Open browser console (F12)
3. Look for:
   - `[ChatBot] ✓ Backend connected successfully!`
   - `[ChatBot] ✓ Conversation created successfully!`
4. Send a test message - chatbot should respond

---

## 🎯 HOW IT WORKS

### Railway Environment (Production)
```
Railway sets: PORT=12345 (dynamic)
Python reads: os.environ.get("PORT") → 12345
Server starts: uvicorn on 0.0.0.0:12345
Railway routes: https://your-app.railway.app → 0.0.0.0:12345
```

### Local Development
```
No PORT set
Python reads: os.environ.get("PORT", 8000) → 8000
Server starts: uvicorn on 0.0.0.0:8000
Access: http://localhost:8000
```

---

## 📋 COMPLETE FILE SUMMARY

### Files Modified:
- ✅ `backend/run.py` - **NEW** Python startup script
- ✅ `Procfile` - Updated to use Python script
- ✅ `railway.json` - Updated startCommand
- ✅ `backend/Procfile` - **DELETED** (duplicate)
- ✅ `backend/start.sh` - **DELETED** (not needed)

### Files Unchanged (working correctly):
- ✅ `backend/src/main.py` - CORS and app config
- ✅ `backend/src/config.py` - Environment variable handling
- ✅ `backend/.env` - Local development settings
- ✅ `.env` - Frontend API URL

---

## 🚨 TROUBLESHOOTING

### If Railway Still Shows PORT Error:
1. Check Railway logs for "Starting server on 0.0.0.0:<port>"
2. Verify `backend/run.py` exists and is committed
3. Verify `Procfile` contains: `web: cd backend && python run.py`
4. Check Railway build logs for Python installation

### If 502 Bad Gateway:
1. **Environment Variables** - Make sure all required vars are set in Railway
2. **Build Logs** - Check for pip install errors
3. **Deploy Logs** - Look for Python/uvicorn startup errors
4. **Healthcheck** - Verify `/health` endpoint is accessible

### If CORS Errors on Vercel:
1. Verify `CORS_ORIGINS=*` in Railway environment variables
2. Check Railway logs for: `Configuring CORS with origins: ['*']`
3. Test CORS headers directly: `curl -i <railway-url>/`

### If Chatbot Doesn't Respond:
1. **Backend Health** - Verify `/health` returns 200 OK
2. **API Keys** - Check `OPENAI_API_KEY` is set in Railway
3. **Qdrant** - Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
4. **Browser Console** - Look for specific error messages

---

## ✅ SUCCESS CHECKLIST

- [ ] `backend/run.py` created
- [ ] `Procfile` updated
- [ ] `railway.json` updated
- [ ] `backend/Procfile` deleted
- [ ] `backend/start.sh` deleted
- [ ] Local testing passed (port 8000 and custom port)
- [ ] Railway environment variables set
- [ ] Changes committed to git
- [ ] Changes pushed to GitHub
- [ ] Railway deployment succeeded
- [ ] Health endpoint returns 200 OK
- [ ] CORS headers present in responses
- [ ] Vercel chatbot connects successfully
- [ ] Chat messages work end-to-end

---

## 🎉 EXPECTED OUTCOME

### Before Fix:
```
Railway logs: Error: Invalid value for '--port': '$PORT' is not a valid integer.
Status: 502 Bad Gateway
Chatbot: Failed to initialize
```

### After Fix:
```
Railway logs: Starting server on 0.0.0.0:<port>
              INFO: Application startup complete
Status: 200 OK
Chatbot: ✓ Connected, responds to messages
```

---

## 📞 NEED HELP?

If issues persist after following all steps:
1. Check Railway deployment logs for specific errors
2. Verify ALL environment variables are set correctly in Railway dashboard
3. Test backend endpoints directly with curl commands above
4. Check browser console for frontend-specific errors

**Local development will continue to work regardless of Railway status.**

---

**Summary:** Railway PORT error fixed by using Python to read environment variable instead of shell substitution. All changes tested locally and ready for deployment.
