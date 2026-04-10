#!/usr/bin/env python3
"""Direct backend startup without uvicorn CLI"""
import os
import sys

# Auto-detect environment and set correct path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
os.chdir(current_dir)

import uvicorn
from main import app

if __name__ == "__main__":
    port = int(os.getenv("PORT", 7860 if os.path.exists("/.dockerenv") else 8001))
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )
