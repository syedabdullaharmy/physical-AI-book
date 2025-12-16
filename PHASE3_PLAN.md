# Phase 3: RAG Chatbot Implementation Plan

## Objective
Build a robust RAG (Retrieval-Augmented Generation) chatbot backend that uses the completed textbook content to answer student questions intelligently.

## Technology Stack
- **Backend Framework:** FastAPI (Python)
- **Database (Vector):** Qdrant (for semantic search)
- **Database (Relational):** Neon Postgres (for chat history/user data)
- **LLM:** Mistral AI (via API)
- **Embeddings:** HuggingFace / Mistral Embeddings
- **Frontend Integration:** React Chat Widget (in Docusaurus)

## Step-by-Step Implementation

### 1. Backend Setup
- [ ] Initialize `backend/` directory structure
- [ ] Create `requirements.txt` with dependencies
- [ ] Set up environment variables (`.env`)

### 2. Database Configuration
- [ ] Set up Qdrant client connection
- [ ] Set up Neon Postgres connection (SQLAlchemy/AsyncPG)
- [ ] Define database models (ChatHistory, Users)

### 3. Document Ingestion Pipeline
- [ ] Create `ingest.py` script
- [ ] **Reader:** Parse Markdown files from `frontend/docs`
- [ ] **Splitter:** Chunk text intelligently (preserving code blocks and headers)
- [ ] **Embedder:** Generate vector embeddings for chunks
- [ ] **Store:** Upload vectors to Qdrant collection `textbook`

### 4. RAG Logic (Chat Endpoint)
- [ ] Create `/api/chat` endpoint
- [ ] **Retrieve:** Search Qdrant for relevant chunks based on user query
- [ ] **Augment:** Construct prompt with Context + Query
- [ ] **Generate:** Call Mistral AI API for response
- [ ] **Store:** Save conversation to Postgres

### 5. Frontend Integration
- [ ] Create `ChatWidget.tsx` component in Docusaurus
- [ ] Add floating chat button
- [ ] Connect widget to backend API

## Next Action
Start by setting up the backend structure and dependencies.
