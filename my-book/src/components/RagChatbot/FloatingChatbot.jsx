import React, { useState, useRef, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import './ChatStyles.css';
import { submitQuery } from '../../services/rag-api';
import { saveChatSession, loadChatSession, isStorageAvailable, clearChatSession } from '../../utils/storage-utils';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
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

  // Toggle chat window open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat window
  const closeChat = () => {
    setIsOpen(false);
  };

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  // Function to handle pasting selected text
  const handlePasteSelectedText = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      // You can either add the selected text to the input field or send it directly as a query
      // For now, we'll just add it to the input field by triggering an event
      // In a real implementation, we might want to add it to the input field of ChatInput
      return selectedText;
    }
    return '';
  };

  return (
    <>
      {/* Floating chat button */}
      <button
        className={`floating-chat-button ${isOpen ? 'hidden' : ''}`}
        onClick={toggleChat}
        aria-label="Open AI Assistant"
        title="AI Assistant"
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM8 16L10 14L8 12L9 11L11 13L13 11L14 12L12 14L14 16L13 17L11 15L9 17L8 16ZM15 15L17 13L15 11L16 10L18 12L20 10L21 11L19 13L21 15L20 16L18 14L16 16L15 15Z" fill="currentColor"/>
        </svg>
      </button>

      {/* Chat popup window */}
      {isOpen && (
        <div className="floating-chat-popup">
          <div className="floating-chat-header">
            <h3>AI Assistant</h3>
            <button
              className="close-button"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
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
            selectedText={handlePasteSelectedText()}
          />
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;