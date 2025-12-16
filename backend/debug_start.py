import sys
try:
    print("Importing app.main...")
    from app.main import app
    print("app.main imported successfully")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
