import os
import sys

# Add the 'backend' directory to the Python path
# This allows us to import 'app' from 'backend/app'
sys.path.append(os.path.join(os.path.dirname(__file__), '../backend'))

from app.main import app
