# Quick Deployment Script for Vercel

Write-Host "🚀 Deploying to Vercel..." -ForegroundColor Cyan

# Check if we're in the right directory
if (-not (Test-Path "vercel.json")) {
    Write-Host "❌ Error: vercel.json not found. Please run this script from the project root." -ForegroundColor Red
    exit 1
}

# Stage all changes
Write-Host "`n📦 Staging changes..." -ForegroundColor Yellow
git add .

# Commit changes
Write-Host "`n💾 Committing changes..." -ForegroundColor Yellow
$commitMessage = Read-Host "Enter commit message (or press Enter for default)"
if ([string]::IsNullOrWhiteSpace($commitMessage)) {
    $commitMessage = "Fix: Vercel chatbot connection routing"
}
git commit -m $commitMessage

# Push to repository
Write-Host "`n⬆️  Pushing to repository..." -ForegroundColor Yellow
git push origin main

Write-Host "`n✅ Deployment initiated! Check your Vercel dashboard for deployment status." -ForegroundColor Green
Write-Host "`n📝 Don't forget to:" -ForegroundColor Cyan
Write-Host "   1. Set environment variables in Vercel dashboard" -ForegroundColor White
Write-Host "   2. Wait for deployment to complete" -ForegroundColor White
Write-Host "   3. Test the chatbot on your live site" -ForegroundColor White
Write-Host "`n🔗 Vercel Dashboard: https://vercel.com/dashboard" -ForegroundColor Cyan
