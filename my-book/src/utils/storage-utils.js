/**
 * Storage utilities for chat persistence
 */

const CHAT_STORAGE_KEY = 'rag-chat-history';
const MAX_SESSIONS = 5; // Keep only the last 5 sessions

/**
 * Save chat session to localStorage
 * @param {string} sessionId - The session ID
 * @param {Array} messages - The messages array to save
 */
export const saveChatSession = (sessionId, messages) => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return; // Skip if localStorage is not available
    }

    // Don't save error messages to reduce storage bloat
    const messagesToSave = messages.filter(msg =>
      msg.type !== 'system' || !msg.error
    );

    if (messagesToSave.length === 0) {
      return; // Don't save empty sessions
    }

    const sessionData = {
      id: sessionId,
      messages: messagesToSave,
      timestamp: Date.now()
    };

    // Get existing sessions
    const existingSessions = getStoredSessions();

    // Add new session
    const updatedSessions = [sessionData, ...existingSessions.filter(s => s.id !== sessionId)];

    // Keep only the most recent sessions
    const sessionsToKeep = updatedSessions.slice(0, MAX_SESSIONS);

    // Save to localStorage
    localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify(sessionsToKeep));
  } catch (error) {
    console.warn('Failed to save chat session to localStorage:', error);
  }
};

/**
 * Load chat session from localStorage
 * @param {string} sessionId - The session ID to load
 * @returns {Array|null} The messages array or null if not found
 */
export const loadChatSession = (sessionId) => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return null; // Skip if localStorage is not available
    }

    const sessions = getStoredSessions();
    const session = sessions.find(s => s.id === sessionId);

    return session ? session.messages : null;
  } catch (error) {
    console.warn('Failed to load chat session from localStorage:', error);
    return null;
  }
};

/**
 * Get all stored sessions
 * @returns {Array} Array of stored sessions
 */
export const getStoredSessions = () => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return [];
    }

    const stored = localStorage.getItem(CHAT_STORAGE_KEY);
    return stored ? JSON.parse(stored) : [];
  } catch (error) {
    console.warn('Failed to read chat sessions from localStorage:', error);
    return [];
  }
};

/**
 * Clear stored sessions for a specific session ID
 * @param {string} sessionId - The session ID to clear
 */
export const clearChatSession = (sessionId) => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return;
    }

    const sessions = getStoredSessions();
    const updatedSessions = sessions.filter(s => s.id !== sessionId);
    localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify(updatedSessions));
  } catch (error) {
    console.warn('Failed to clear chat session from localStorage:', error);
  }
};

/**
 * Clear all stored sessions
 */
export const clearAllChatSessions = () => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return;
    }

    localStorage.removeItem(CHAT_STORAGE_KEY);
  } catch (error) {
    console.warn('Failed to clear all chat sessions from localStorage:', error);
  }
};

/**
 * Check if storage is available and working
 * @returns {boolean} Whether storage is available
 */
export const isStorageAvailable = () => {
  try {
    if (typeof window === 'undefined' || !window.localStorage) {
      return false;
    }

    // Test if we can set and get an item
    const testKey = '__storage_test__';
    window.localStorage.setItem(testKey, 'test');
    window.localStorage.removeItem(testKey);
    return true;
  } catch (e) {
    return false;
  }
};