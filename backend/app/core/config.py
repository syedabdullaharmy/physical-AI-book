import os
from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    PROJECT_NAME: str = "Physical AI Textbook Chatbot"
    API_V1_STR: str = "/api/v1"
    
    # Mistral AI
    MISTRAL_API_KEY: str
    
    # Qdrant
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION: str = "textbook_rag"
    
    # Postgres
    POSTGRES_USER: str = "user"
    POSTGRES_PASSWORD: str = "password"
    POSTGRES_SERVER: str = "localhost"
    POSTGRES_DB: str = "textbook_db"
    POSTGRES_PORT: str = "5432"
    
    USE_SQLITE: bool = False
    
    SECRET_KEY: str = "SECRET_KEY_Should_Be_Start_With_Env_If_Possible"
    
    @property
    def DATABASE_URL(self) -> str:
        if self.USE_SQLITE:
            # On Vercel, usage of absolute path /tmp is required for writing
            return "sqlite+aiosqlite:////tmp/test.db"
        return f"postgresql+asyncpg://{self.POSTGRES_USER}:{self.POSTGRES_PASSWORD}@{self.POSTGRES_SERVER}:{self.POSTGRES_PORT}/{self.POSTGRES_DB}"

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
