try:
    from langchain_core.documents import Document
    print("langchain_core import successful")
except ImportError as e:
    print(f"langchain_core import failed: {e}")
    try:
        from langchain.docstore.document import Document
        print("langchain.docstore import successful")
    except ImportError as e2:
        print(f"langchain.docstore import failed: {e2}")

