import requests
import json

try:
    print("Testing backend health...")
    r = requests.get("http://127.0.0.1:8000/health")
    print(f"Health status: {r.status_code} {r.text}")

    print("\nTesting chat endpoint...")
    url = "http://127.0.0.1:8000/api/v1/chat"
    payload = {"message": "hello", "history": []}
    
    with requests.post(url, json=payload, stream=True) as r:
        print(f"Chat status: {r.status_code}")
        if r.status_code == 200:
            print("Response chunks:")
            for chunk in r.iter_content(chunk_size=None):
                if chunk:
                    print(chunk.decode('utf-8'), end='', flush=True)
        else:
            print(f"Error: {r.text}")

except Exception as e:
    print(f"\nTest failed: {e}")
