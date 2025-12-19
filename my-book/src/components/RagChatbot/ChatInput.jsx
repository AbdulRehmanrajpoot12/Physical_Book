import React, { useState, useRef } from 'react';
import { submitQuery } from '../../services/rag-api';
import { validateQuery, sanitizeInput } from '../../utils/chat-utils';

const ChatInput = ({ onSend, isLoading, sessionId, context, onError, onLoadingChange, selectedText }) => {
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef(null);

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) {
      return;
    }

    try {
      // Sanitize input to prevent XSS
      const sanitizedInput = sanitizeInput(inputValue);

      // Validate input length (min 1 char, max 1000 chars)
      const validation = validateQuery(sanitizedInput);
      if (!validation.isValid) {
        onError(validation.error);
        return;
      }

      // Create user message object
      const userMessage = {
        id: `msg_${Date.now()}_user`,
        type: 'user',
        content: sanitizedInput.trim(),
        timestamp: new Date()
      };

      // Add user message to chat
      onSend(userMessage);
      setInputValue('');
      onLoadingChange(true);
      onError(null); // Clear any previous errors

      // Submit query to RAG agent
      const response = await submitQuery(inputValue.trim());

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
      onSend(systemMessage);
    } catch (err) {
      console.error('Error submitting query:', err);
      onError(`Failed to get response: ${err.message || 'Unknown error'}`);

      // Add error message to chat as a system message
      const errorMessage = {
        id: `msg_${Date.now()}_error`,
        type: 'system',
        content: `Sorry, I encountered an error processing your request: ${err.message || 'Unknown error'}`,
        timestamp: new Date(),
        error: err,
        canRetry: true // Flag to indicate this error message has a retry option
      };

      onSend(errorMessage);
    } finally {
      onLoadingChange(false);
    }
  };

  const handlePasteSelectedText = () => {
    if (selectedText) {
      setInputValue(prev => prev + (prev ? ' ' : '') + selectedText);
      // Focus the textarea and move cursor to the end
      setTimeout(() => {
        if (textareaRef.current) {
          textareaRef.current.focus();
          textareaRef.current.setSelectionRange(textareaRef.current.value.length, textareaRef.current.value.length);
        }
      }, 0);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="chat-input-form" role="form" aria-label="Chat message input form">
      <div style={{ display: 'flex', flexDirection: 'column', flex: '1' }}>
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the book..."
          disabled={isLoading}
          className="chat-input"
          rows="1"
          aria-label="Type your message here"
          aria-disabled={isLoading}
          onKeyPress={(e) => {
            // Allow Shift+Enter for new lines, but submit on Enter (without Shift)
            if (e.key === 'Enter' && !e.shiftKey) {
              e.preventDefault();
              handleSubmit(e);
            }
          }}
          style={{
            flex: '1',
            minHeight: '40px',
            maxHeight: '100px',
            overflowY: 'auto'
          }}
        />
      </div>
      <div style={{ display: 'flex', gap: '4px' }}>
        {selectedText && (
          <button
            type="button"
            onClick={handlePasteSelectedText}
            className="chat-paste-btn"
            aria-label="Paste selected text"
            title="Paste selected text"
            style={{
              padding: '8px',
              border: '1px solid #ddd',
              borderRadius: '4px',
              cursor: 'pointer',
              backgroundColor: '#f5f5f5'
            }}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2H4C3.47 2 3 2.24 3 2.5V3H2C1.47 3 1 3.24 1 3.5V14.5C1 14.76 1.47 15 2 15H12C12.53 15 13 14.76 13 14.5V2.5C13 2.24 12.53 2 12 2ZM11 4V13H3V4H11ZM5 6H6V7H5V6ZM5 8H9V9H5V8ZM5 10H9V11H5V10Z" fill="currentColor"/>
            </svg>
          </button>
        )}
        <button
          type="submit"
          disabled={isLoading || !inputValue.trim()}
          className="chat-submit-btn"
          aria-label={isLoading ? 'Sending message, please wait' : 'Send message'}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </form>
  );
};

export default ChatInput;