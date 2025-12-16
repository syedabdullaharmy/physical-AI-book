@echo off
echo Starting FastAPI server...
uvicorn app.main:app --reload
pause
