# API Setup Guide

This guide explains how to set up the required API keys for the RAG chatbot application.

## Required API Keys

### 1. Neon PostgreSQL Database (DATABASE_URL)
- **Validation**: Must be a valid PostgreSQL connection string
- **Setup**:
  - Create an account at [Neon](https://neon.tech/)
  - Create a new project
  - Copy the connection string from the project dashboard
- **Format**: `postgresql://user:pass@ep-xxx.us-east-1.aws.neon.tech/neondb?sslmode=require`
- **Required**: Yes

### 2. Better Auth Secret (BETTER_AUTH_SECRET)
- **Validation**: Must be at least 32 characters
- **Setup**:
  - Generate a 32+ character secret key
  - Use a password generator or command like:
    ```bash
    openssl rand -base64 32
    ```
- **Required**: Yes

### 3. Better Auth URL (BETTER_AUTH_URL)
- **Validation**: Must be a valid URL
- **Setup**: The URL where your application will be hosted
- **Default for development**: `http://localhost:3000`
- **Required**: Yes

### 4. Qdrant Vector Database
- **QDRANT_URL Validation**: Must be a valid URL
  - Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
  - Create a new cluster
  - Get the **Cluster Endpoint** from the cluster dashboard
  - **Format**: `https://xxx.us-east-1.aws.cloud.qdrant.io:6333`
- **QDRANT_API_KEY Validation**: Required field
  - Get the API key from the cluster dashboard (in the "API Keys" section)
- **Required**: Yes

### 5. OpenAI API Key (OPENAI_API_KEY)
- **Validation**: Must start with `sk-`
- **Setup**:
  - Create an account at [OpenAI](https://platform.openai.com/)
  - Generate an API key in the dashboard
- **Usage**: Only used for generating text embeddings (text-embedding-3-small model)
- **Required**: Yes

### 6. Google Gemini API Key (GEMINI_API_KEY)
- **Validation**: Required field
- **Setup**:
  - Get an API key from [Google AI Studio](https://aistudio.google.com/)
- **Usage**: Used for chat completions (gemini-2.5-flash model)
- **Required**: Yes

### 7. Resend API Key (RESEND_API_KEY)
- **Validation**: Must start with `re_`
- **Setup**:
  - Sign up at [Resend](https://resend.com/)
  - Generate an API key in the dashboard
- **EMAIL_FROM Validation**: Must be a valid email address
  - Set to your verified domain or Resend's sandbox domain
- **Required**: Yes (for email verification and password reset)

## Environment File Setup

1. Copy the `.env.example` file to `.env`:
   ```bash
   cp .env.example .env
   ```

2. Replace all placeholder values with your actual API keys following the validation requirements above

3. **Important**: Never commit the `.env` file to version control (it's already in `.gitignore`)

## Running the Application

After setting up your API keys:

1. Install dependencies:
   ```bash
   npm install
   ```

2. Run database migrations:
   ```bash
   npm run db:push
   ```

3. Start the development server:
   ```bash
   npm run dev
   ```

The application will be available at `http://localhost:3000`

## Troubleshooting

If you encounter validation errors:
- Check that all API keys meet the validation requirements specified above
- Ensure no extra spaces or quotes around the values in your `.env` file
- Restart the development server after making changes to `.env`

## Testing Environment Variables

To verify your environment variables are properly set:

1. Run the type checker:
   ```bash
   npx tsc --noEmit
   ```

2. Try building the application:
   ```bash
   npm run build
   ```

3. Check for validation errors in the console during startup

## Security Best Practices

- Never commit `.env` files to version control
- Use different API keys for development and production
- Rotate API keys periodically
- Restrict API key permissions to minimum required access
- Use environment-specific variables (e.g., `NEXT_PUBLIC_BASE_URL`)