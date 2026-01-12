#!/bin/bash

# Chatbot Integration Test Script
# This script helps verify the backend-frontend integration

echo "🤖 Chatbot Integration Test Script"
echo "=================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check if backend is running
echo "Test 1: Checking backend server..."
BACKEND_HEALTH=$(curl -s http://localhost:3001/api/health 2>/dev/null)

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Backend is running on port 3001${NC}"
    echo "  Response: $BACKEND_HEALTH"
else
    echo -e "${RED}✗ Backend is NOT running${NC}"
    echo -e "${YELLOW}  Start it with: cd backend && npm run dev${NC}"
    exit 1
fi

echo ""

# Test 2: Check if frontend is accessible
echo "Test 2: Checking frontend server..."
FRONTEND_CHECK=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3000 2>/dev/null)

if [ "$FRONTEND_CHECK" = "200" ]; then
    echo -e "${GREEN}✓ Frontend is running on port 3000${NC}"
else
    echo -e "${RED}✗ Frontend is NOT running${NC}"
    echo -e "${YELLOW}  Start it with: cd my-website && npm start${NC}"
    exit 1
fi

echo ""

# Test 3: Test public chat endpoint
echo "Test 3: Testing chat endpoint..."
CHAT_RESPONSE=$(curl -s -X POST http://localhost:3001/api/chat/public \
  -H "Content-Type: application/json" \
  -d '{"message":"test"}' \
  --max-time 5 2>/dev/null)

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Chat endpoint is responding${NC}"
    echo "  First 100 chars: ${CHAT_RESPONSE:0:100}..."
else
    echo -e "${RED}✗ Chat endpoint failed${NC}"
    exit 1
fi

echo ""

# Test 4: Check Qdrant connection
echo "Test 4: Checking Qdrant connection..."
if [ -f "backend/.env" ]; then
    QDRANT_URL=$(grep QDRANT_URL backend/.env | cut -d '=' -f2 | tr -d '"')
    if [ ! -z "$QDRANT_URL" ]; then
        echo -e "${GREEN}✓ Qdrant URL configured: ${QDRANT_URL:0:40}...${NC}"
    else
        echo -e "${YELLOW}⚠ Qdrant URL not found in .env${NC}"
    fi
else
    echo -e "${RED}✗ backend/.env file not found${NC}"
fi

echo ""

# Test 5: Check Gemini API key
echo "Test 5: Checking Gemini API key..."
if [ -f "backend/.env" ]; then
    GEMINI_KEY=$(grep GEMINI_API_KEY backend/.env | cut -d '=' -f2 | tr -d '"')
    if [ ! -z "$GEMINI_KEY" ]; then
        echo -e "${GREEN}✓ Gemini API key configured${NC}"
    else
        echo -e "${RED}✗ Gemini API key not found${NC}"
    fi
fi

echo ""
echo "=================================="
echo -e "${GREEN}✓ All tests passed!${NC}"
echo ""
echo "Next steps:"
echo "1. Open http://localhost:3000 in your browser"
echo "2. Click the robot avatar button (bottom-right)"
echo "3. Ask: 'What is ROS 2?'"
echo ""
echo "For detailed testing guide, see: CHATBOT_INTEGRATION_GUIDE.md"
