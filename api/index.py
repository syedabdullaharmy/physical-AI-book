import os
import sys
from mangum import Mangum

# Add the 'backend' directory to the Python path
# This allows us to import 'app' from 'backend/app'
sys.path.append(os.path.join(os.path.dirname(__file__), '../backend'))

from app.main import app

# Wrap the ASGI app with Mangum for serverless deployment
handler = Mangum(app)
