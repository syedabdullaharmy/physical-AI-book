import os
import glob
from typing import List
from langchain_community.document_loaders import TextLoader
from langchain_text_splitters import MarkdownHeaderTextSplitter, RecursiveCharacterTextSplitter
from langchain_core.documents import Document
from mistralai.client import MistralClient
from qdrant_client import QdrantClient
from qdrant_client.http import models
from app.core.config import settings

class IngestionService:
    def __init__(self):
        self.docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../frontend/docs"))
        # Using Mistral API for embeddings
        self.mistral = MistralClient(api_key=settings.MISTRAL_API_KEY)
        self.vector_size = 1024
        
        # Initialize Qdrant
        self.qdrant = QdrantClient(
            url=settings.QDRANT_URL, 
            api_key=settings.QDRANT_API_KEY
        )

    def load_documents(self) -> List[Document]:
        """Load all markdown files from the docs directory"""
        documents = []
        # Find all .md files recursively
        md_files = glob.glob(os.path.join(self.docs_path, "**/*.md"), recursive=True)
        
        print(f"Found {len(md_files)} markdown files in {self.docs_path}")
        
        for file_path in md_files:
            try:
                loader = TextLoader(file_path, encoding='utf-8')
                docs = loader.load()
                
                # Add metadata
                for doc in docs:
                    # Create relative path for source link
                    rel_path = os.path.relpath(file_path, self.docs_path)
                    doc.metadata["source"] = rel_path
                    doc.metadata["filename"] = os.path.basename(file_path)
                    # Add meaningful module info based on path
                    path_parts = rel_path.split(os.sep)
                    if len(path_parts) > 1 and "module" in path_parts[0]:
                        doc.metadata["module"] = path_parts[0]
                
                documents.extend(docs)
            except Exception as e:
                print(f"Error loading {file_path}: {e}")
                
        return documents

    def split_documents(self, documents: List[Document]) -> List[Document]:
        """Split documents into semantic chunks"""
        
        # 1. Split by Markdown Headers first to keep context
        headers_to_split_on = [
            ("#", "Header 1"),
            ("##", "Header 2"),
            ("###", "Header 3"),
        ]
        
        markdown_splitter = MarkdownHeaderTextSplitter(headers_to_split_on=headers_to_split_on)
        
        md_docs = []
        for doc in documents:
            splits = markdown_splitter.split_text(doc.page_content)
            for split in splits:
                split.metadata.update(doc.metadata)
                md_docs.append(split)
                
        # 2. Recursive split for size
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            separators=["\n\n", "\n", " ", ""]
        )
        
        return text_splitter.split_documents(md_docs)

    def ingest(self):
        """Run the full ingestion pipeline"""
        print("Starting ingestion...")
        
        # 1. Load
        raw_docs = self.load_documents()
        print(f"Loaded {len(raw_docs)} documents.")
        
        # 2. Split
        chunks = self.split_documents(raw_docs)
        print(f"Created {len(chunks)} chunks.")
        
        # 3. Re-create Collection
        self.qdrant.recreate_collection(
            collection_name=settings.QDRANT_COLLECTION,
            vectors_config=models.VectorParams(
                size=self.vector_size, 
                distance=models.Distance.COSINE
            )
        )
        print(f"Recreated collection '{settings.QDRANT_COLLECTION}'")
        
        # 4. Embed and Upsert in batches
        batch_size = 100
        total_chunks = len(chunks)
        
        for i in range(0, total_chunks, batch_size):
            batch = chunks[i : i + batch_size]
            
            # Generate embeddings via Mistral API
            texts = [doc.page_content for doc in batch]
            embeddings_batch = self.mistral.embeddings(
                model="mistral-embed",
                input=texts
            )
            embeddings = [d.embedding for d in embeddings_batch.data]
            
            # Prepare payload
            points = []
            for j, doc in enumerate(batch):
                payload = doc.metadata.copy()
                payload["page_content"] = doc.page_content
                
                points.append(models.PointStruct(
                    id=i + j,
                    vector=embeddings[j],
                    payload=payload
                ))
            
            # Upload
            self.qdrant.upsert(
                collection_name=settings.QDRANT_COLLECTION,
                points=points
            )
            print(f"Processed batch {i // batch_size + 1}/{(total_chunks + batch_size - 1) // batch_size}")
            
        print("Ingestion complete!")

if __name__ == "__main__":
    service = IngestionService()
    service.ingest()
