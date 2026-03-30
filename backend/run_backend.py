#!/usr/bin/env python3
"""Direct backend startup without uvicorn CLI"""
import os
import sys
import asyncio

# Ensure backend is in path
backend_path = r"C:\Users\dell\Desktop\robot-book\backend"
if backend_path not in sys.path:
    sys.path.insert(0, backend_path)

os.chdir(backend_path)

import uvicorn
from main import app

if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8001,  # Use 8001 since 8000 is taken by frontend
        log_level="info"
    )
