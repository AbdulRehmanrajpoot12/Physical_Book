import React from 'react';

const ChatMessage = ({ message, onRetry }) => {
  const { type, content, sources, timestamp, canRetry, error } = message;

  // Format the timestamp for display
  const formatTime = (date) => {
    if (!date) return '';
    const d = new Date(date);
    return d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={`message ${type}-message`} role="listitem" aria-label={`${type} message`}>
      <div className="message-content">
        <p>{content}</p>
        {sources && sources.length > 0 && (
          <div className="sources" role="note" aria-label="Sources">
            <small>Sources: {sources.join(', ')}</small>
          </div>
        )}
        {message.retrievedContexts && Array.isArray(message.retrievedContexts) && message.retrievedContexts.length > 0 && (
          <div className="retrieved-contexts" role="region" aria-label="Retrieved context information">
            <details style={{ marginTop: '8px', fontSize: '0.85em' }}>
              <summary style={{ cursor: 'pointer', color: '#2563eb' }} aria-label="Toggle retrieved context information">
                Show retrieved context ({message.retrievedContexts.length} sources)
              </summary>
              {message.retrievedContexts.map((context, index) => (
                <div key={context.id || index} style={{ marginTop: '8px', padding: '6px', backgroundColor: '#f9fafb', border: '1px solid #e5e7eb', borderRadius: '4px' }}>
                  <div style={{ fontWeight: 'bold', marginBottom: '4px' }}>Source {index + 1} (Score: {context.score ? context.score.toFixed(2) : 'N/A'})</div>
                  <div>{context.content ? context.content.substring(0, 200) + (context.content.length > 200 ? '...' : '') : ''}</div>
                  {context.metadata && Object.keys(context.metadata).length > 0 && (
                    <div style={{ fontSize: '0.8em', color: '#6b7280', marginTop: '4px' }}>
                      Metadata: {JSON.stringify(context.metadata)}
                    </div>
                  )}
                </div>
              ))}
            </details>
          </div>
        )}
        {canRetry && onRetry && (
          <div className="retry-option" role="group" aria-label="Retry options">
            <button
              onClick={onRetry}
              style={{
                marginTop: '5px',
                padding: '4px 8px',
                fontSize: '0.8em',
                backgroundColor: '#2563eb',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
              aria-label="Retry this request"
            >
              Retry
            </button>
          </div>
        )}
        {timestamp && (
          <div className="timestamp" role="note" aria-label="Message timestamp">
            <small style={{ fontSize: '0.7em', color: '#9ca3af', display: 'block' }}>
              {formatTime(timestamp)}
            </small>
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatMessage;