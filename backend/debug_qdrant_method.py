from qdrant_client import QdrantClient
from app.core.config import settings
import sys

try:
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
    print(f"Client type: {type(client)}")
    print("Methods available:")
    attrs = dir(client)
    for a in attrs:
        if 'search' in a:
            print(f" - {a}")
            
    if hasattr(client, 'search'):
        print("SUCCESS: 'search' method found.")
    else:
        print("FAILURE: 'search' method NOT found.")
        # Print all public methods to see alternatives
        print([m for m in attrs if not m.startswith('_')])

except Exception as e:
    print(f"Error: {e}")
