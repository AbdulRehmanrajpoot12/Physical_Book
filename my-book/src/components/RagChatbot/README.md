# RAG Chatbot Component

A React-based floating chatbot widget that integrates with a RAG (Retrieval-Augmented Generation) agent backend to provide contextual answers to user queries about book content.

## Features

- Floating chat widget that appears on every page as a bottom-right button
- Real-time chat interface with message history
- Loading indicators and typing animations
- Error handling with retry functionality
- Context-aware responses with source attribution
- Expandable retrieved context sections
- Session management with automatic cleanup
- Local storage persistence for chat history
- Ability to paste selected text from the page into the chat
- Full accessibility support (ARIA labels, keyboard navigation)

## Usage

The chatbot appears as a floating button on the bottom-right corner of every page. When clicked, it opens a small chat window where users can type questions or paste selected text from the book to get AI explanations. The widget is available on all pages including modules 1-4.

## Technical Details

- **Frontend**: React components built for Docusaurus
- **Backend Communication**: Axios-based API service for RAG agent integration
- **State Management**: React hooks for local state
- **Styling**: CSS modules for scoped styling
- **Data Persistence**: LocalStorage for chat history
- **Security**: Input sanitization to prevent XSS, only safe RAG_AGENT_URL exposed to frontend

## Architecture

```
FloatingChatbot.jsx (main floating widget component)
├── ChatMessage.jsx (displays individual messages)
├── ChatInput.jsx (handles user input and submission with selected text paste functionality)
├── rag-api.js (API service for backend communication)
├── chat-utils.js (utility functions)
├── storage-utils.js (local storage management)
└── ChatStyles.css (CSS for both floating and regular chat interfaces)
```

## API Integration

The component communicates with the RAG agent backend via the following endpoints:
- `POST /query` - Submit user queries and receive contextual responses
- `GET /health` - Check backend service status
- `GET/POST /config` - Manage configuration settings

## Configuration

The RAG agent URL can be configured via the `RAG_AGENT_URL` environment variable in your `.env.local` file or defaults to `http://localhost:8000`.

To configure environment variables:
1. Create a `.env.local` file in the project root
2. Set `RAG_AGENT_URL` to point to your RAG agent backend
3. The environment variable will be injected into the frontend at build time via Docusaurus configuration

**Security Note**: Only the safe `RAG_AGENT_URL` is exposed to the frontend. Sensitive API keys (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY) should be configured on the backend service and not exposed to the frontend for security reasons.

## Error Handling

- Network timeouts are handled gracefully with user-friendly messages
- Server errors display appropriate error messages with retry options
- Input validation prevents empty or overly long queries
- Session timeouts automatically clear old conversations
- Backend unavailability is handled with clear error messages

## Accessibility

- Floating button with proper ARIA labels
- Keyboard navigable interface
- Proper semantic HTML structure
- Focus management for dynamic content
- Screen reader support for all interactive elements

## Performance

- Session cleanup after 24 hours of inactivity
- Message history limited to prevent excessive memory usage
- Efficient re-rendering with React's virtual DOM
- Lazy loading of chat history