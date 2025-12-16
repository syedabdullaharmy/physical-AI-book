import sys
import time

def log(msg):
    print(f"[{time.strftime('%X')}] {msg}")

try:
    log("Importing fastapi")
    import fastapi
    log("Importing qdrant_client")
    import qdrant_client
    log("Importing mistralai")
    import mistralai
    log("Importing langchain_community.document_loaders")
    from langchain_community.document_loaders import TextLoader
    log("Importing langchain_text_splitters")
    from langchain_text_splitters import MarkdownHeaderTextSplitter
    log("Importing app.core.config")
    from app.core.config import settings
    log("Importing app.services.rag")
    from app.services.rag import RAGService
    log("Importing app.services.ingestion")
    from app.services.ingestion import IngestionService
    log("All imports successful")
except Exception as e:
    log(f"Error: {e}")
    import traceback
    traceback.print_exc()
