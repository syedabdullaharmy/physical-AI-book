# Physical AI Textbook - Chatbot Backend

This directory contains the FastAPI backend for the RAG (Retrieval-Augmented Generation) chatbot.

## Setup

1.  **Install Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

2.  **Configuration:**
    - Rename `.env.example` to `.env`
    - Update `MISTRAL_API_KEY` with your key.
    - Update `QDRANT_URL` (Default `http://localhost:6333` requires Qdrant Docker).
      - If you don't have Docker, you can set `QDRANT_URL=:memory:` for a temporary in-memory vector DB (data lost on restart) or a local path like `path/to/db`.

3.  **Run Server:**
    ```bash
    uvicorn app.main:app --reload
    ```
    The API will be available at `http://localhost:8000`.
    Documentation: `http://localhost:8000/docs`.

## Ingestion

To populate the vector database with the textbook content, run:

```bash
python -m app.services.ingestion
```

Or convert the content via API:
```bash
curl -X POST http://localhost:8000/api/v1/ingest
```

## Architecture

- **FastAPI:** Web server.
- **Qdrant:** Vector database for semantic search.
- **Mistral AI:** LLM for generating answers.
- **LangChain:** Text splitting and processing.
