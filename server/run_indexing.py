import sys
import os

# Add the server directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'app')))

from services.indexing_service import run_indexing

if __name__ == "__main__":
    run_indexing()
