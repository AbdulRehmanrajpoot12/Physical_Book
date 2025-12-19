/**
 * API service for communicating with the RAG agent backend
 */
import axios from 'axios';

// Configuration for RAG agent backend URL
// Use window-based configuration that can be set by Docusaurus via HTML template or global variables
const RAG_AGENT_URL =
  (typeof window !== 'undefined' && window.RAG_CONFIG && window.RAG_CONFIG.RAG_AGENT_URL)
    ? window.RAG_CONFIG.RAG_AGENT_URL
    : 'http://localhost:8000';

// Create axios instance with base configuration
const apiClient = axios.create({
  baseURL: RAG_AGENT_URL,
  timeout: 30000, // 30 second timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

/**
 * Submit a query to the RAG agent
 * @param {string} query - The user's question/query
 * @param {number} top_k - Number of results to retrieve (default: 5)
 * @param {boolean} include_metadata - Whether to include metadata (default: true)
 * @returns {Promise<Object>} The RAG agent response
 */
export const submitQuery = async (query, top_k = 5, include_metadata = true) => {
  try {
    const response = await apiClient.post('/query', {
      query,
      top_k,
      include_metadata
    });

    return response.data;
  } catch (error) {
    console.error('Error submitting query to RAG agent:', error);

    // Handle different types of errors
    if (axios.isCancel(error)) {
      // Request was cancelled
      throw new Error('Request was cancelled');
    } else if (error.response) {
      // Server responded with error status
      throw new Error(`API request failed: ${error.response.status} - ${error.response.data?.error || error.response.statusText}`);
    } else if (error.request) {
      // Request was made but no response received (likely timeout or network issue)
      if (error.code === 'ECONNABORTED' || error.code === 'ETIMEDOUT') {
        throw new Error('Request timed out. The RAG agent service may be slow to respond. Please try again.');
      } else {
        throw new Error('Network error: Unable to reach the RAG agent service. Please check your connection and try again.');
      }
    } else {
      // Something else happened
      throw new Error(error.message || 'Unknown error occurred while submitting query');
    }
  }
};

/**
 * Check if the RAG agent service is healthy
 * @returns {Promise<boolean>} Whether the service is healthy
 */
export const checkHealth = async () => {
  try {
    const response = await apiClient.get('/health');
    return response.status === 200;
  } catch (error) {
    console.error('Error checking RAG agent health:', error);
    return false;
  }
};

/**
 * Get configuration from the RAG agent
 * @returns {Promise<Object>} The current configuration
 */
export const getConfig = async () => {
  try {
    const response = await apiClient.get('/config');
    return response.data;
  } catch (error) {
    console.error('Error getting RAG agent config:', error);

    if (error.response) {
      throw new Error(`Failed to get config: ${error.response.status} - ${error.response.data?.error || error.response.statusText}`);
    } else {
      throw error;
    }
  }
};

/**
 * Update configuration in the RAG agent
 * @param {Object} config - The configuration to update
 * @returns {Promise<Object>} The updated configuration
 */
export const updateConfig = async (config) => {
  try {
    const response = await apiClient.post('/config', config);
    return response.data;
  } catch (error) {
    console.error('Error updating RAG agent config:', error);

    if (error.response) {
      throw new Error(`Failed to update config: ${error.response.status} - ${error.response.data?.error || error.response.statusText}`);
    } else {
      throw error;
    }
  }
};