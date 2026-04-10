#!/usr/bin/env python3
import os
import sys
import uvicorn

# Set paths
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
os.chdir(current_dir)

from main import app

if __name__ == "__main__":
    # Stable startup without reload to prevent Windows process issues
    port = int(os.getenv("PORT", 8001))
    print(f"🚀 Starting Robot Book Backend on http://127.0.0.1:{port}")
    uvicorn.run(
        app,
        host="127.0.0.1",
        port=port,
        log_level="info"
    )
