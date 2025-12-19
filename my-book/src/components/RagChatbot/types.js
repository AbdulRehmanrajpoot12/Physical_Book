/**
 * Type definitions for RAG Chatbot entities
 * This file documents the expected structure of data objects used in the chatbot
 */

// Query: Represents a user's question or request sent to the RAG agent
/**
 * @typedef {Object} Query
 * @property {string} id - Unique identifier for the query
 * @property {string} text - The actual query text from the user
 * @property {Date} timestamp - When the query was submitted
 * @property {('pending'|'processing'|'completed'|'error')} status - Query status
 */

// Response: Represents the RAG agent's answer to a query
/**
 * @typedef {Object} Response
 * @property {string} id - Unique identifier matching the query ID
 * @property {string} queryId - Reference to the original query
 * @property {string} answer - The main answer text from the RAG agent
 * @property {Array<RetrievedContext>} retrievedContexts - List of context objects used to generate the response
 * @property {Array<string>} sources - List of source identifiers
 * @property {Date} timestamp - When the response was received
 * @property {('success'|'error'|'timeout')} status - Response status
 */

// RetrievedContext: Represents a piece of context used by the RAG agent to generate a response
/**
 * @typedef {Object} RetrievedContext
 * @property {string} id - Unique identifier for the context item
 * @property {string} content - The actual content text
 * @property {number} score - Relevance score (0.0 - 1.0)
 * @property {Object} metadata - Additional metadata about the source
 */

// ChatMessage: Represents a message in the chat interface (either user query or system response)
/**
 * @typedef {Object} ChatMessage
 * @property {string} id - Unique identifier for the message
 * @property {('user'|'system')} type - Message type
 * @property {string} content - The message content
 * @property {Date} timestamp - When the message was created
 * @property {Array<string>} [sources] - Optional sources for system responses
 */

// ChatSession: Represents a conversation session between user and RAG agent
/**
 * @typedef {Object} ChatSession
 * @property {string} id - Unique identifier for the session
 * @property {Array<ChatMessage>} messages - Array of ChatMessage objects in chronological order
 * @property {Date} createdAt - When the session was created
 * @property {Date} lastActiveAt - When the session was last used
 */

// LoadingState: Represents the loading state of the chat interface
/**
 * @typedef {Object} LoadingState
 * @property {boolean} isLoading - Whether a query is currently being processed
 * @property {string} [message] - Optional message to display during loading
 * @property {number} [progress] - Optional progress indicator (0-100)
 */

// ErrorState: Represents error conditions in the chat interface
/**
 * @typedef {Object} ErrorState
 * @property {boolean} hasError - Whether an error occurred
 * @property {string} message - User-friendly error message
 * @property {string} code - Error code for debugging
 * @property {Date} timestamp - When the error occurred
 */

// Export empty object to make this a valid JS module
export {};