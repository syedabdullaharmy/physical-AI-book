from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional
from app.services.rag import RAGService
from app.services.ingestion import IngestionService

router = APIRouter()
rag_service = RAGService()

class Message(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    history: Optional[List[Message]] = []

@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that streams the response from the LLM
    """
    return StreamingResponse(
        rag_service.generate_answer(request.message, [h.model_dump() for h in request.history]),
        media_type="text/event-stream"
    )

@router.post("/ingest")
async def ingest_endpoint():
    """
    Trigger document ingestion manually
    """
    try:
        service = IngestionService()
        service.ingest()
        return {"status": "success", "message": "Ingestion completed"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
