# Research: Docusaurus Book Setup

## Decision: Docusaurus Version and Template Choice
**Rationale**: Using Docusaurus v3 with the classic template provides a well-structured documentation site with built-in features like search, versioning, and mobile responsiveness. The classic template is ideal for book-style content with multiple chapters and modules.
**Alternatives considered**:
- Custom React site: More complex, requires more setup
- MkDocs: Different ecosystem, less React-focused
- GitBook: Different toolchain, less customization options

## Decision: Project Structure for Modules and Chapters
**Rationale**: Organizing content in a hierarchical structure (module-1/chapter-name.md) makes it easy to navigate and maintain. This follows Docusaurus best practices for documentation sites.
**Alternatives considered**:
- Flat structure: Harder to organize as the book grows
- Deep nesting: Could make navigation complex

## Decision: GitHub Pages Deployment Strategy
**Rationale**: GitHub Pages provides free hosting with custom domains and is tightly integrated with Git workflows. Using the gh-pages branch is the standard approach for Docusaurus sites.
**Alternatives considered**:
- Netlify/Vercel: Additional services, though more features
- Self-hosting: More complex infrastructure

## Decision: Markdown Format for Content
**Rationale**: Markdown is the standard for documentation and integrates well with Docusaurus. It's readable in plain text and supports rich formatting when rendered.
**Alternatives considered**:
- MDX: More powerful but adds complexity
- RestructuredText: Different syntax, less familiar to developers

## Best Practices Researched
1. Use consistent heading hierarchy (h1 for page titles, h2-h6 for sections)
2. Include metadata at the top of each document
3. Use relative links between pages
4. Optimize images for web delivery
5. Use Docusaurus admonitions for notes, warnings, and tips
6. Structure sidebar navigation logically

## ROS 2 Content Structure Research
For the three planned chapters:
- ROS 2 Basics: Installation, concepts, nodes, topics, services
- Python Agents & rclpy: Client libraries, node implementation, pub/sub patterns
- URDF: Robot modeling, XML format, visualization, transforms