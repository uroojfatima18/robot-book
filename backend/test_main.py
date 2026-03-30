#!/usr/bin/env python3
import traceback
try:
    import main
    print('✅ main module imports successfully!')
    print(f'app: {main.app}')
except Exception as e:
    print(f'❌ Error importing main:')
    traceback.print_exc()
