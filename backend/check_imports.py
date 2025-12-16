try:
    from mistralai.client import MistralClient
    print("Client import ok")
    from mistralai.models.chat_completion import ChatMessage
    print("ChatMessage import ok")
except ImportError as e:
    print(f"Import failed: {e}")
