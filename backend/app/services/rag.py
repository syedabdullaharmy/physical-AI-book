from typing import List, Dict, Generator
from qdrant_client import QdrantClient

from mistralai.client import MistralClient
from mistralai.models.chat_completion import ChatMessage
from app.core.config import settings

class RAGService:
    def __init__(self):

        self.qdrant = QdrantClient(
            url=settings.QDRANT_URL, 
            api_key=settings.QDRANT_API_KEY
        )
        self.mistral = MistralClient(api_key=settings.MISTRAL_API_KEY) if settings.MISTRAL_API_KEY else None
    
    def retrieve_context(self, query: str, top_k: int = 3) -> List[Dict]:
        """Retrieve relevant context chunks from Qdrant"""
        # Generate query embedding using Mistral
        embed_response = self.mistral.embeddings(
            model="mistral-embed",
            input=[query]
        )
        query_vector = embed_response.data[0].embedding
        
        search_result = self.qdrant.query_points(
            collection_name=settings.QDRANT_COLLECTION,
            query=query_vector,
            limit=top_k
        ).points
        
        results = []
        for hit in search_result:
            results.append({
                "content": hit.payload.get("page_content", ""),
                "source": hit.payload.get("source", "unknown"),
                "score": hit.score
            })
        
        return results

    def generate_answer(self, query: str, history: List[Dict] = []) -> Generator[str, None, None]:
        """Generate answer using RAG and Mistral"""
        
        try:
            # 1. Retrieve Context
            context_items = self.retrieve_context(query)
            context_str = "\n\n".join([f"--- Source: {item['source']} ---\n{item['content']}" for item in context_items])
        except Exception as e:
            print(f"Error retrieving context: {e}")
            yield f"System Error: Failed to retrieve knowledge base context. ({str(e)})"
            return
        
        # 2. Construct System Prompt
        system_prompt = f"""You are an expert AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook.
Use the following context to answer the student's question accurately. 
If the answer is not in the context, say you don't know but try to be helpful based on general robotics knowledge, 
while explicitly stating what is and isn't covered in the provided text.

Context:
{context_str}
"""

        # 3. Prepare Messages
        messages = [ChatMessage(role="system", content=system_prompt)]
        
        # Add history (last 5 turns to keep context window manageable)
        for msg in history[-5:]:
            role = msg.get("role", "user")
            content = msg.get("content", "")
            messages.append(ChatMessage(role=role, content=content))
            
        # Add current query
        messages.append(ChatMessage(role="user", content=query))
        
        # 4. Call LLM
        if not self.mistral:
            yield "Mistral API Key is missing. Please configure backend credentials."
            return

        try:
            # Reverting to non-streaming to fix "Attempted to access streaming response content" error
            # The current mistralai library version seems to have issues with the stream generator wrapper
            chat_response = self.mistral.chat(
                model="mistral-medium",
                messages=messages,
            )
            
            if chat_response.choices and chat_response.choices[0].message.content:
                # Yield the full content as a single chunk since we are not streaming
                yield chat_response.choices[0].message.content
                    
        except Exception as e:
            print(f"Mistral Error: {e}")
            yield f"Error generating response: {str(e)}"
