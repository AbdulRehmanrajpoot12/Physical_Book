# Data Model: RAG Agent Frontend Integration

**Feature**: 007-rag-frontend-integration
**Date**: 2025-12-19
**Status**: Complete

## Overview

This document defines the data structures used in the RAG Agent Frontend Integration feature, including frontend state management and data flow between components.

## Frontend Data Entities

### 1. Query
**Purpose**: Represents a user's question or request sent to the RAG agent

**Fields**:
- `id` (string): Unique identifier for the query
- `text` (string): The actual query text from the user
- `timestamp` (Date): When the query was submitted
- `status` (enum): Query status ('pending', 'processing', 'completed', 'error')

**Validation**:
- `text` must be non-empty (min length: 1 character)
- `text` must not exceed 1000 characters

### 2. Response
**Purpose**: Represents the RAG agent's answer to a query

**Fields**:
- `id` (string): Unique identifier matching the query ID
- `queryId` (string): Reference to the original query
- `answer` (string): The main answer text from the RAG agent
- `retrievedContexts` (array): List of context objects used to generate the response
- `sources` (array): List of source identifiers
- `timestamp` (Date): When the response was received
- `status` (enum): Response status ('success', 'error', 'timeout')

**Validation**:
- `answer` must be non-empty when status is 'success'
- `retrievedContexts` must be an array of context objects

### 3. RetrievedContext
**Purpose**: Represents a piece of context used by the RAG agent to generate a response

**Fields**:
- `id` (string): Unique identifier for the context item
- `content` (string): The actual content text
- `score` (number): Relevance score (0.0 - 1.0)
- `metadata` (object): Additional metadata about the source

**Validation**:
- `content` must be non-empty
- `score` must be between 0.0 and 1.0

### 4. ChatMessage
**Purpose**: Represents a message in the chat interface (either user query or system response)

**Fields**:
- `id` (string): Unique identifier for the message
- `type` (enum): Message type ('user', 'system')
- `content` (string): The message content
- `timestamp` (Date): When the message was created
- `sources` (array): Optional sources for system responses

**Validation**:
- `content` must be non-empty
- `type` must be either 'user' or 'system'

### 5. ChatSession
**Purpose**: Represents a conversation session between user and RAG agent

**Fields**:
- `id` (string): Unique identifier for the session
- `messages` (array): Array of ChatMessage objects in chronological order
- `createdAt` (Date): When the session was created
- `lastActiveAt` (Date): When the session was last used

**Validation**:
- `messages` must be an array
- `messages` length should not exceed 100 (to prevent memory issues)

### 6. LoadingState
**Purpose**: Represents the loading state of the chat interface

**Fields**:
- `isLoading` (boolean): Whether a query is currently being processed
- `message` (string): Optional message to display during loading
- `progress` (number): Optional progress indicator (0-100)

### 7. ErrorState
**Purpose**: Represents error conditions in the chat interface

**Fields**:
- `hasError` (boolean): Whether an error occurred
- `message` (string): User-friendly error message
- `code` (string): Error code for debugging
- `timestamp` (Date): When the error occurred

## API Data Contracts

### Frontend to Backend Request
```
{
  "query": "string (required)",
  "top_k": "number (optional, default: 5)",
  "include_metadata": "boolean (optional, default: true)"
}
```

### Backend to Frontend Response
```
{
  "query": "string",
  "answer": "string",
  "retrieved_contexts": [
    {
      "id": "string",
      "content": "string",
      "score": "number",
      "metadata": "object"
    }
  ],
  "sources": "string[]"
}
```

## State Management Schema

### Chat Component State
```
{
  "chatSession": ChatSession,
  "loadingState": LoadingState,
  "errorState": ErrorState,
  "config": {
    "topK": number,
    "includeMetadata": boolean
  }
}
```

## Data Validation Rules

1. **Query Validation**:
   - Minimum length: 1 character
   - Maximum length: 1000 characters
   - Must contain non-whitespace characters

2. **Response Validation**:
   - Answer must be present when status is 'success'
   - Retrieved contexts must be properly formatted
   - Sources must be an array of strings

3. **Session Management**:
   - Maximum 100 messages per session to prevent memory issues
   - Automatic session cleanup after 24 hours of inactivity

## Relationships

- `ChatSession` contains multiple `ChatMessage` objects
- `ChatMessage` of type 'system' may reference multiple `RetrievedContext` objects
- `Query` and `Response` are linked by matching IDs
- `ErrorState` is associated with a specific `ChatSession`

## Constraints

1. **Size Limits**:
   - Individual message content: 5000 characters max
   - Session message history: 100 messages max
   - Query text: 1000 characters max

2. **Performance**:
   - Response time should be under 10 seconds
   - UI should remain responsive during API calls

3. **Security**:
   - No sensitive data stored client-side
   - Input sanitization required for user queries