# 📋 Vercel Deployment Checklist

## Pre-Deployment Checklist

### ✅ Code Changes (Completed)
- [x] Fixed `vercel.json` routing configuration
- [x] Updated API URL configuration in `AuthContext.tsx`
- [x] Added `aiosqlite` to `requirements.txt`
- [x] Created `.env.example` template
- [x] Updated `.gitignore` to exclude sensitive files
- [x] Created deployment documentation

### 🔑 Environment Variables Setup (Required)

Go to: **Vercel Dashboard → Your Project → Settings → Environment Variables**

#### Required Variables
- [ ] `MISTRAL_API_KEY` - Get from [Mistral AI Console](https://console.mistral.ai/)
- [ ] `QDRANT_URL` - Get from [Qdrant Cloud](https://cloud.qdrant.io/)
- [ ] `QDRANT_API_KEY` - Get from [Qdrant Cloud](https://cloud.qdrant.io/)
- [ ] `SECRET_KEY` - Generate a secure random string (32+ characters)

#### Database Variables (Choose one option)

**Option A: PostgreSQL (Recommended for Production)**
- [ ] `POSTGRES_USER`
- [ ] `POSTGRES_PASSWORD`
- [ ] `POSTGRES_SERVER`
- [ ] `POSTGRES_DB`
- [ ] `POSTGRES_PORT` (usually 5432)
- [ ] `USE_SQLITE=false`

**Option B: SQLite (For Testing Only)**
- [ ] `USE_SQLITE=true`

#### Optional Variables
- [ ] `QDRANT_COLLECTION` (default: textbook_rag)
- [ ] `API_V1_STR` (default: /api/v1)
- [ ] `PROJECT_NAME` (default: Physical AI Textbook Chatbot)

### 📦 Deployment Steps

#### Step 1: Commit Changes
```bash
git add .
git commit -m "Fix: Vercel chatbot connection routing"
```

#### Step 2: Push to Repository
```bash
git push origin main
```

#### Step 3: Monitor Deployment
- [ ] Go to Vercel Dashboard
- [ ] Watch the deployment progress
- [ ] Check for any build errors
- [ ] Wait for "Ready" status

### 🧪 Post-Deployment Testing

#### Step 1: Basic Functionality
- [ ] Visit your Vercel site URL
- [ ] Verify the site loads correctly
- [ ] Check that all pages are accessible

#### Step 2: Chatbot Testing
- [ ] Click the chatbot button (bottom-right corner)
- [ ] Chatbot window opens successfully
- [ ] Send a test message: "What is Physical AI?"
- [ ] Receive a response from the AI assistant
- [ ] No error messages appear

#### Step 3: Authentication Testing
- [ ] Navigate to login page
- [ ] Try logging in (if you have an account)
- [ ] Try signing up for a new account
- [ ] Verify authentication works correctly

#### Step 4: Feature Testing
- [ ] Test content personalization features
- [ ] Test Urdu translation functionality
- [ ] Test bookmark functionality
- [ ] Test chapter progress tracking

### 🔍 Troubleshooting Guide

#### If Chatbot Shows Connection Error:

1. **Check Vercel Function Logs**
   - Vercel Dashboard → Deployments → Latest → Functions
   - Look for error messages

2. **Verify Environment Variables**
   - Settings → Environment Variables
   - Ensure all required variables are set
   - Check for typos

3. **Check Browser Console**
   - Press F12 → Console tab
   - Look for JavaScript errors
   - Check Network tab for failed requests

4. **Common Issues:**
   - [ ] Missing environment variables → Add them and redeploy
   - [ ] Invalid API keys → Verify credentials
   - [ ] Database connection failed → Check database URL
   - [ ] CORS errors → Check backend CORS configuration

### 📊 Success Criteria

Your deployment is successful when:

- [x] Site loads without errors
- [x] Chatbot connects and responds
- [x] Authentication works
- [x] All features are functional
- [x] No console errors
- [x] API requests return 200 OK

### 🎯 Quick Commands

#### Generate a secure SECRET_KEY:
```powershell
# PowerShell
-join ((48..57) + (65..90) + (97..122) | Get-Random -Count 32 | ForEach-Object {[char]$_})
```

#### Check deployment status:
```bash
vercel ls
```

#### View logs:
```bash
vercel logs
```

### 📞 Support Resources

- **Vercel Documentation:** https://vercel.com/docs
- **Mistral AI Docs:** https://docs.mistral.ai/
- **Qdrant Docs:** https://qdrant.tech/documentation/
- **FastAPI Docs:** https://fastapi.tiangolo.com/

### 🎉 After Successful Deployment

1. **Share your site** with users
2. **Monitor usage** in Vercel Analytics
3. **Collect feedback** from users
4. **Iterate and improve** based on feedback

---

**Status:** Ready to deploy! 🚀
**Estimated Time:** 10-15 minutes
**Difficulty:** Easy

Good luck with your deployment! 🎊
