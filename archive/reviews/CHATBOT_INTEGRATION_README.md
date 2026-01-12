# Docuaurus Chatbot Integration

This project integrates a custom chatbot widget into the Docuaurus website. The chatbot communicates with the Next.js chat API to provide AI-powered responses about robotics topics.

## Project Structure

- `src/components/ChatbotWidget.tsx` - The main chatbot UI component
- `src/utils/chatbot-api.js` - API utility for communicating with the chat backend
- `src/theme/Root.tsx` - Docuaurus theme component where the chatbot is integrated
- `chatbot-app/` - The Next.js application containing the chat API

## Setup Instructions

### 1. Environment Configuration

Create a `.env` file in the `my-website` directory:

```bash
NEXT_PUBLIC_CHAT_API_URL=http://localhost:3000/api/chat/public
```

### 2. Running the Applications

You need to run both the Docuaurus site and the Next.js chat API:

#### Option A: Separate Terminals

1. **Start the Next.js chat API:**
   ```bash
   cd my-website/chatbot-app
   npm install
   npm run dev
   ```
   The API will be available at `http://localhost:3000/api/chat/public`

2. **Start the Docuaurus site:**
   ```bash
   cd my-website
   npm install
   npx docusaurus start
   ```
   The site will be available at `http://localhost:3000`

#### Option B: Using Concurrently (Recommended)

1. Install concurrently globally:
   ```bash
   npm install -g concurrently
   ```

2. Run both applications:
   ```bash
   cd my-website
   concurrently "cd chatbot-app && npm run dev" "npx docusaurus start"
   ```

### 3. Production Build

To build both applications for production:

1. **Build the Next.js chat API:**
   ```bash
   cd my-website/chatbot-app
   npm run build
   ```

2. **Build the Docuaurus site:**
   ```bash
   cd my-website
   npm run build
   ```

## Configuration

### API Endpoint Configuration

The chatbot API endpoint can be configured via environment variables:

- `NEXT_PUBLIC_CHAT_API_URL` - The URL of the chat API endpoint
  - Development: `http://localhost:3000/api/chat/public`
  - Production: Your deployed API URL

### Chatbot Widget Features

- Floating chat widget in the bottom-right corner
- Persistent conversation history using localStorage
- Streaming responses from the AI
- Error handling and display
- Responsive design for all screen sizes

## Troubleshooting

### Common Issues

1. **CORS Errors**: Make sure both applications are running on compatible origins or that CORS is properly configured in the Next.js app.

2. **API Not Found**: Verify that the Next.js chat API is running and accessible at the configured URL.

3. **Environment Variables**: Ensure that environment variables are properly set and loaded.

### API Endpoints

The chatbot expects the following API endpoint:
- `POST /api/chat/public` - Send a message and receive a streaming response

## Development

### Adding New Features

1. Modify the chatbot UI in `src/components/ChatbotWidget.tsx`
2. Update API communication in `src/utils/chatbot-api.js`
3. Test thoroughly in both development and production builds

### Testing the Integration

1. Start both applications as described above
2. Navigate to the Docuaurus site
3. Verify the chatbot widget appears in the bottom-right corner
4. Test sending messages and receiving responses
5. Verify conversation history persists across page reloads