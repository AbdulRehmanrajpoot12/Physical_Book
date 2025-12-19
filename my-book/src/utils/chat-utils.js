/**
 * Utility functions for chat functionality
 */

/**
 * Generate a unique ID
 * @param {string} prefix - Optional prefix for the ID
 * @returns {string} A unique identifier
 */
export const generateId = (prefix = 'id') => {
  return `${prefix}_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Get current timestamp
 * @returns {Date} Current date and time
 */
export const getCurrentTimestamp = () => {
  return new Date();
};

/**
 * Format timestamp for display
 * @param {Date} date - The date to format
 * @returns {string} Formatted time string
 */
export const formatTime = (date) => {
  if (!date) return '';
  const d = new Date(date);
  return d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
};

/**
 * Validate query text
 * @param {string} text - The query text to validate
 * @returns {Object} Validation result with isValid and error message
 */
export const validateQuery = (text) => {
  if (typeof text !== 'string') {
    return { isValid: false, error: 'Query must be a string' };
  }

  if (text.trim().length < 1) {
    return { isValid: false, error: 'Query must contain at least 1 character' };
  }

  if (text.trim().length > 1000) {
    return { isValid: false, error: 'Query must be less than 1000 characters' };
  }

  return { isValid: true, error: null };
};

/**
 * Sanitize user input to prevent XSS
 * @param {string} input - The input to sanitize
 * @returns {string} Sanitized input
 */
export const sanitizeInput = (input) => {
  if (typeof input !== 'string') {
    return '';
  }

  // Basic XSS prevention - remove script tags and other potentially dangerous elements
  return input
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '')
    .replace(/javascript:/gi, '')
    .replace(/on\w+\s*=/gi, '');
};

/**
 * Truncate text to specified length
 * @param {string} text - The text to truncate
 * @param {number} maxLength - Maximum length
 * @param {string} suffix - Suffix to add if truncated (default: '...')
 * @returns {string} Truncated text
 */
export const truncateText = (text, maxLength, suffix = '...') => {
  if (!text || typeof text !== 'string') {
    return '';
  }

  if (text.length <= maxLength) {
    return text;
  }

  return text.substring(0, maxLength - suffix.length) + suffix;
};

/**
 * Format sources for display
 * @param {Array} sources - Array of source identifiers
 * @returns {string} Formatted sources string
 */
export const formatSources = (sources) => {
  if (!Array.isArray(sources) || sources.length === 0) {
    return '';
  }

  return sources.join(', ');
};

/**
 * Format retrieved contexts for display
 * @param {Array} contexts - Array of retrieved context objects
 * @returns {Array} Formatted context objects
 */
export const formatRetrievedContexts = (contexts) => {
  if (!Array.isArray(contexts)) {
    return [];
  }

  return contexts.map(ctx => ({
    id: ctx.id,
    content: ctx.content ? truncateText(ctx.content, 200) : '',
    score: ctx.score || 0,
    metadata: ctx.metadata || {}
  }));
};