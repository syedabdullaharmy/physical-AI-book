try:
    from app.services.ingestion import IngestionService
    print("IngestionService imported")
except ImportError as e:
    print(f"Error: {e}")
