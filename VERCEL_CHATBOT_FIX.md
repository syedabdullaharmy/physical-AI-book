# Vercel Chatbot Connection Fix

## Problem
The chatbot on Vercel was showing the error: "Could not connect to AI backend. Please ensure the backend server is running."

## Root Cause
The issue was with the Vercel routing configuration. The frontend was making requests to `/api/v1/*` endpoints, but Vercel wasn't properly routing these requests to the Python serverless function.

## Solution Applied

### 1. Updated `vercel.json` Routing
Added explicit routing for `/api/v1/*` requests to ensure they reach the Python backend:

```json
"rewrites": [
    {
        "source": "/api/v1/(.*)",
        "destination": "/api/index.py"
    },
    {
        "source": "/api/(.*)",
        "destination": "/api/index.py"
    },
    {
        "source": "/(.*)",
        "destination": "/frontend/$1"
    }
]
```

### 2. Improved API URL Configuration
Updated `frontend/src/contexts/AuthContext.tsx` to support environment variables and have proper fallbacks.

## Deployment Steps

### Step 1: Set Environment Variables in Vercel

Go to your Vercel project settings and add these environment variables:

**Required:**
- `MISTRAL_API_KEY` - Your Mistral AI API key
- `QDRANT_URL` - Your Qdrant cloud URL
- `QDRANT_API_KEY` - Your Qdrant API key
- `SECRET_KEY` - A secure random string (minimum 32 characters)

**Database (Choose one):**

**Option A: PostgreSQL (Recommended for production)**
- `POSTGRES_USER` - PostgreSQL username
- `POSTGRES_PASSWORD` - PostgreSQL password
- `POSTGRES_SERVER` - PostgreSQL server URL
- `POSTGRES_DB` - Database name
- `POSTGRES_PORT` - Port (usually 5432)
- `USE_SQLITE` - Set to `false`

**Option B: SQLite (For testing only)**
- `USE_SQLITE` - Set to `true`

**Optional:**
- `QDRANT_COLLECTION` - Collection name (default: `textbook_rag`)
- `API_V1_STR` - API prefix (default: `/api/v1`)
- `PROJECT_NAME` - Project name (default: `Physical AI Textbook Chatbot`)

### Step 2: Deploy to Vercel

```bash
# Commit the changes
git add .
git commit -m "Fix: Vercel chatbot connection routing"

# Push to your repository
git push origin main
```

Vercel will automatically redeploy your application.

### Step 3: Verify the Deployment

1. Wait for the deployment to complete
2. Visit your Vercel site
3. Open the chatbot widget
4. Send a test message
5. You should receive a response from the AI assistant

## Testing Locally

To test the changes locally before deploying:

```bash
# Install dependencies
cd frontend
npm install

# Build the frontend
npm run build

# Test the production build
npm run serve
```

## Troubleshooting

### If the chatbot still doesn't work:

1. **Check Vercel Logs:**
   - Go to your Vercel dashboard
   - Click on your deployment
   - Check the "Functions" tab for any errors

2. **Verify Environment Variables:**
   - Ensure all required environment variables are set
   - Check for typos in variable names
   - Redeploy after adding/updating variables

3. **Check API Endpoint:**
   - Open browser DevTools (F12)
   - Go to Network tab
   - Try sending a message
   - Check if the request to `/api/v1/chat` returns 200 OK

4. **CORS Issues:**
   - The backend is configured to allow all origins in development
   - For production, you may need to restrict this

5. **Database Connection:**
   - If using PostgreSQL, ensure the connection string is correct
   - If using SQLite, ensure `USE_SQLITE=true` is set

## Next Steps

After deployment, you should:

1. ✅ Test the chatbot functionality
2. ✅ Verify user authentication works
3. ✅ Check that content personalization features work
4. ✅ Test the Urdu translation feature
5. ✅ Monitor Vercel function logs for any errors

## Files Modified

- `vercel.json` - Added explicit `/api/v1/*` routing
- `frontend/src/contexts/AuthContext.tsx` - Improved API URL configuration
- `.env.example` - Created environment variables template

## Support

If you continue to experience issues:

1. Check the Vercel function logs
2. Verify all environment variables are correctly set
3. Ensure your Mistral AI and Qdrant credentials are valid
4. Check that your database is accessible from Vercel's servers
