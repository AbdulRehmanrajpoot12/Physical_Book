<!-- CONSTITUTION_FINALIZED: DO NOT MODIFY -->

Project: AI-Authored Technical Book with Embedded RAG Chatbot

Objective:
Create a spec-driven technical book using Docusaurus, authored with Claude Code and Spec-Kit Plus, deployed to GitHub Pages, and enhanced with an embedded RAG chatbot for contextual Q&A over the book content.

---

Core Principles:

- Spec-first development (no implementation without specification)
- Accuracy and faithfulness to source content
- Reproducibility of build and data pipelines
- Clear, technical writing for developers and CS students
- Zero hallucination tolerance in AI outputs

---

Book Standards:

- Platform: Docusaurus (Markdown/MDX)
- Deployment: GitHub Pages
- Tone: Book-style, instructional, precise
- Each chapter must include structured sections and a summary

---

AI Authoring Rules (Claude Code):

- Follow chapter specs exactly
- Do not invent APIs, tools, or references
- Ask for clarification on ambiguity
- No conversational filler or emojis

---

RAG Chatbot Requirements:

- Backend: FastAPI
- LLM: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- Metadata DB: Neon Serverless Postgres

Capabilities:

1. Answer questions using full book content
2. Answer questions strictly from user-selected text
3. If answer is not in retrieved context, respond with uncertainty

Rules:

- Every answer must be grounded in retrieved content
- Source section/chapter must be identifiable
- Hallucination is a critical failure

---

Data Pipeline:

- Markdown/MDX is the single source of truth
- Deterministic chunking preserving headings and code blocks
- Embeddings must be reproducible from raw content

---

Security:

- No client-side API keys
- No storage of user queries without consent

---

Success Criteria:

- Book builds and deploys on GitHub Pages
- RAG chatbot answers accurately and transparently
- Context-scoped Q&A works correctly
- Project is fully reproducible from repository
