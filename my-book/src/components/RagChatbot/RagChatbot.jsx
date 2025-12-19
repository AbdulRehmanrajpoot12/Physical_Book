import React, { useState, useRef, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import './ChatStyles.css';
import { submitQuery } from '../../services/rag-api';
import { saveChatSession, loadChatSession, isStorageAvailable, clearChatSession } from '../../utils/storage-utils';

// Define TypeScript-like interfaces as JavaScript objects for reference
// Query: { id: string, text: string, timestamp: Date, status: 'pending' | 'processing' | 'completed' | 'error' }
// Response: { id: string, queryId: string, answer: string, retrievedContexts: Array, sources: Array, timestamp: Date, status: 'success' | 'error' | 'timeout' }
// ChatMessage: { id: string, type: 'user' | 'system', content: string, timestamp: Date, sources?: Array }
// ChatSession: { id: string, messages: Array<ChatMessage>, createdAt: Date, lastActiveAt: Date }

const RagChatbot = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);
  const [sessionId] = useState(() => {
    // Create a unique session ID for this chat session
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  });

  // Initialize messages with potential saved session data
  const [messages, setMessages] = useState(() => {
    // Try to load any existing session data
    if (isStorageAvailable()) {
      const savedMessages = loadChatSession(sessionId);
      if (savedMessages && Array.isArray(savedMessages) && savedMessages.length > 0) {
        return savedMessages;
      }
    }

    // Return empty array if no saved data or storage not available
    return [];
  });

  // Track last activity time for session cleanup
  const [lastActivity, setLastActivity] = useState(Date.now());

  // Session timeout: 24 hours in milliseconds
  const SESSION_TIMEOUT = 24 * 60 * 60 * 1000; // 24 hours

  // Check for session timeout periodically
  useEffect(() => {
    const checkSessionTimeout = () => {
      const now = Date.now();
      if (now - lastActivity > SESSION_TIMEOUT) {
        // Session has expired, reset the chat
        setMessages([]);
        setError(null);

        // Clear the saved session from localStorage
        if (isStorageAvailable()) {
          clearChatSession(sessionId);
        }

        // Add a new welcome message
        setMessages([
          {
            id: `msg_${Date.now()}_welcome`,
            type: 'system',
            content: 'Your session has expired. Starting a new chat session. Ask me anything about the book content!',
            timestamp: new Date()
          }
        ]);

        // Update last activity to now
        setLastActivity(now);
      }
    };

    // Check every 5 minutes
    const interval = setInterval(checkSessionTimeout, 5 * 60 * 1000);

    return () => clearInterval(interval);
  }, [lastActivity]);

  // Save messages to localStorage whenever they change
  useEffect(() => {
    if (isStorageAvailable() && messages.length > 0) {
      saveChatSession(sessionId, messages);
    }
  }, [messages, sessionId]);

  // Update last activity when messages change
  useEffect(() => {
    setLastActivity(Date.now());
  }, [messages]);

  // Scroll to bottom of messages when new messages are added
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Add a welcome message when the component mounts (only if no saved messages)
  useEffect(() => {
    // Only show welcome message if there are no saved messages
    if (messages.length === 0) {
      // Add a welcome message after a short delay
      const welcomeTimer = setTimeout(() => {
        setMessages(prev => [
          ...prev,
          {
            id: `msg_${Date.now()}_welcome`,
            type: 'system',
            content: 'Hello! I\'m your AI assistant. Ask me anything about the book content and I\'ll help find relevant information.',
            timestamp: new Date()
          }
        ]);
      }, 500);

      return () => clearTimeout(welcomeTimer);
    }
  }, [messages.length]); // Only run when messages length changes initially

  // Function to add a new message to the chat
  const addMessage = (message) => {
    setMessages(prev => [...prev, message]);
  };

  // Function to update an existing message (useful for updating status)
  const updateMessage = (messageId, updates) => {
    setMessages(prev =>
      prev.map(msg =>
        msg.id === messageId ? { ...msg, ...updates } : msg
      )
    );
  };

  // Function to get the current context for conversation
  const getContext = () => {
    // Return the last few messages as context
    // In a more sophisticated implementation, this could be limited by token count or time
    return messages.slice(-5); // Last 5 messages as context
  };

  // Function to maintain conversation context between queries
  const getConversationHistory = () => {
    // Filter messages to get only user and system messages (excluding errors)
    return messages
      .filter(msg => msg.type === 'user' || msg.type === 'system')
      .map(msg => ({
        type: msg.type,
        content: msg.content,
        timestamp: msg.timestamp
      }));
  };

  // Function to retry a failed message
  const handleRetry = async (messageId) => {
    const messageToRetry = messages.find(msg => msg.id === messageId);
    if (!messageToRetry || messageToRetry.type !== 'user') {
      // If it's not a user message, find the preceding user message to retry
      const messageIndex = messages.findIndex(msg => msg.id === messageId);
      if (messageIndex > 0) {
        const prevMessage = messages[messageIndex - 1];
        if (prevMessage.type === 'user') {
          // Retry the previous user message
          handleRetry(prevMessage.id);
          return;
        }
      }
      return;
    }

    // Remove the error message and re-send the original query
    setMessages(prev => prev.filter(msg => msg.id !== messageId));

    try {
      setIsLoading(true);
      setError(null);

      // Submit query to RAG agent
      const response = await submitQuery(messageToRetry.content);

      // Create system response message
      const systemMessage = {
        id: `msg_${Date.now()}_system`,
        type: 'system',
        content: response.answer,
        sources: response.sources || [],
        retrievedContexts: response.retrieved_contexts || [],
        timestamp: new Date()
      };

      // Add system response to chat
      setMessages(prev => [...prev, systemMessage]);
    } catch (err) {
      console.error('Error retrying query:', err);
      setError(`Failed to retry query: ${err.message || 'Unknown error'}`);

      // Add error message to chat as a system message
      const errorMessage = {
        id: `msg_${Date.now()}_error_retry`,
        type: 'system',
        content: `Sorry, I encountered an error processing your request: ${err.message || 'Unknown error'}`,
        timestamp: new Date(),
        canRetry: true
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div
      className="rag-chatbot-container"
      role="main"
      aria-label="AI Assistant Chat Interface"
    >
      <div className="chat-header" role="banner">
        <h3>AI Assistant</h3>
      </div>

      <div
        className="chat-messages"
        role="log"
        aria-live="polite"
        aria-atomic="false"
      >
        {messages.map((message) => (
          <ChatMessage
            key={message.id}
            message={message}
            onRetry={() => handleRetry(message.id)}
            role="listitem"
          />
        ))}
        {isLoading && (
          <div
            className="message system-message"
            role="status"
            aria-label="Loading response"
          >
            <div className="message-content">
              <div className="typing-indicator">
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
              </div>
            </div>
          </div>
        )}
        {error && (
          <div
            className="message system-message error-message"
            role="alert"
          >
            <div className="message-content">
              <p>{error}</p>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} aria-hidden="true" />
      </div>

      <ChatInput
        onSend={addMessage}
        isLoading={isLoading}
        sessionId={sessionId}
        context={getContext()}
        onError={setError}
        onLoadingChange={setIsLoading}
        role="complementary"
        aria-label="Chat input"
      />
    </div>
  );
};

export default RagChatbot;