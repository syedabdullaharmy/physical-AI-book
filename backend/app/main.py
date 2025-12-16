from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.api import api_router
from app.core.config import settings

from contextlib import asynccontextmanager
from app.db.database import create_db_and_tables

@asynccontextmanager
async def lifespan(app: FastAPI):
    await create_db_and_tables()
    yield

app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json",
    lifespan=lifespan
)

# Set all CORS enabled origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For development, allow all. Restrict in prod.
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def root():
    return {"message": "Welcome to Physical AI Textbook Chatbot API", "status": "active"}

@app.get("/health")
def health_check():
    return {"status": "ok"}

app.include_router(api_router, prefix=settings.API_V1_STR)
