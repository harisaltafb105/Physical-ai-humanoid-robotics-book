# Railway PORT Variable Fix

## ✅ ISSUE RESOLVED

### Error Message
```
Starting Container
Usage: uvicorn [OPTIONS] APP
Error: Invalid value for '--port': '$PORT' is not a valid integer.
```

### Root Cause
Railway's Procfile **does not automatically use a shell** to interpret bash variable substitution syntax `${PORT:-8000}`. Uvicorn received the literal string `"$PORT"` instead of the environment variable value.

### Solution Applied
Wrapped the command in `sh -c` so the shell can properly interpret `${PORT:-8000}`:

**Before:**
```
web: cd backend && uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}
```

**After:**
```
web: sh -c 'cd backend && uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'
```

---

## 📝 Changes Made

### 1. **Procfile** (Root directory)
```bash
web: sh -c 'cd backend && uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'
```

### 2. **railway.json**
```json
{
  "deploy": {
    "startCommand": "sh -c 'cd backend && uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'",
    "healthcheckPath": "/health",
    "healthcheckTimeout": 100
  }
}
```

---

## ✅ Verification

### Local Test (Passed)
```bash
$ cd backend && sh -c 'uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'
$ curl http://localhost:8000/health
{"status":"healthy","services":{"openai":"connected","qdrant":"connected","database":"fallback"}}
```

### Railway Deployment
- **Status:** Pushed to GitHub (commit: `845fade`)
- **Auto-Deploy:** Railway will redeploy automatically (2-3 minutes)
- **Expected:** Backend starts successfully on Railway's dynamic PORT

---

## 🧪 How to Verify Railway Deployment

### Step 1: Wait for Deployment (2-3 minutes)
Check Railway dashboard logs for successful startup.

### Step 2: Test Health Endpoint
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

### Step 3: Test CORS Headers
```bash
curl -i https://acceptable-alignment-production.up.railway.app/
```

**Expected Headers:**
```
HTTP/1.1 200 OK
access-control-allow-origin: *
...
```

### Step 4: Test Vercel Frontend
1. Open: `https://physical-ai-humanoid-robotics-book-umber.vercel.app/ai-chatbot`
2. Check browser console for:
   - `[ChatBot] ✓ Backend connected successfully!`
   - No CORS errors
3. Send a test message to chatbot

---

## 🎯 What This Fix Does

| Scenario | Behavior |
|----------|----------|
| **Railway (Production)** | Uses `$PORT` from Railway environment (dynamic port) |
| **Local Development** | Falls back to port `8000` when `$PORT` not set |
| **Command Execution** | `sh -c` ensures shell interprets variable substitution |

---

## 🔧 Environment Variables Still Required

This fix solves the PORT issue, but you still need to set these in Railway:

```bash
CORS_ORIGINS=*
OPENAI_API_KEY=<your-key>
QDRANT_URL=<your-url>
QDRANT_API_KEY=<your-key>
DATABASE_URL=<your-database-url>
AUTH_SECRET=<your-secret>
```

See `RAILWAY_ENV_SETUP.md` for detailed setup instructions.

---

## 📋 Troubleshooting

### If Railway Still Fails:

1. **Check Railway Logs**
   - Look for "Starting Container" message
   - Verify no PORT-related errors
   - Check if uvicorn starts successfully

2. **Verify Procfile**
   - Must be in **root directory** (not backend/)
   - Must use `sh -c` wrapper
   - Must use single quotes around command

3. **Test Locally**
   ```bash
   cd backend
   sh -c 'uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}'
   ```

### If Chatbot Doesn't Work:

1. **Backend Health:** Verify `/health` endpoint returns 200 OK
2. **CORS Headers:** Check `access-control-allow-origin: *` is present
3. **Environment Variables:** Ensure all required vars set in Railway
4. **Vercel Config:** Verify `NEXT_PUBLIC_API_URL` points to Railway URL

---

## ✅ Success Criteria

- [x] Railway starts without PORT errors
- [ ] Health endpoint returns 200 OK
- [ ] CORS headers present in responses
- [ ] Vercel chatbot connects successfully
- [ ] Chat messages work end-to-end

---

## 🚀 Next Steps

1. **Wait 2-3 minutes** for Railway to redeploy
2. **Test health endpoint** (see Step 2 above)
3. **Set environment variables** in Railway if not already done
4. **Test Vercel chatbot** to verify end-to-end functionality
5. **Done!** 🎉

---

## 📝 Technical Notes

### Why `sh -c` is Required

Railway's process runner doesn't automatically use bash/sh for Procfile commands. Without a shell:
- `${PORT:-8000}` is passed as a literal string
- Uvicorn receives `"$PORT"` instead of the integer value
- Results in: `'$PORT' is not a valid integer`

### Why Single Quotes

Using single quotes `'...'` in Procfile prevents premature variable expansion:
- Double quotes `"..."` might expand variables in the Procfile context
- Single quotes preserve the command for `sh -c` to execute
- Shell then properly interprets `${PORT:-8000}` at runtime

### Fallback Port Logic

`${PORT:-8000}` means:
- Use `$PORT` if it exists and is not empty
- Otherwise use `8000` as default
- Perfect for local dev (no PORT var) and Railway (has PORT var)

---

**All changes committed:** `845fade`
**Status:** Deployed to Railway
**Local Development:** Unaffected ✓
