try:
    from mistralai.client import MistralClient
    print("MistralClient import successful")
except ImportError as e:
    print(f"MistralClient import failed: {e}")

try:
    from mistralai import Mistral
    print("Mistral import successful")
except ImportError as e:
    print(f"Mistral import failed: {e}")

