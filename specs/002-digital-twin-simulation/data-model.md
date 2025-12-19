# Data Model: Digital Twin Simulation for Physical AI & Humanoid Robotics

## Entities

### Chapter Content
- **Name**: Digital Twins for Physical AI
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/configuration examples
- **Relationships**: Part of Module 2
- **Validation rules**: Must include all required sections (objectives, content, summary)

### Chapter Content
- **Name**: Gazebo Physics Simulation
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/configuration examples
- **Relationships**: Part of Module 2
- **Validation rules**: Must include practical examples of Gazebo physics simulation

### Chapter Content
- **Name**: Unity High-Fidelity Interaction
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/configuration examples
- **Relationships**: Part of Module 2
- **Validation rules**: Must include Unity-specific examples and best practices

### Docusaurus Configuration
- **Name**: Site Configuration
- **Fields**:
  - title: string (site title)
  - tagline: string (site tagline)
  - url: string (site URL)
  - baseUrl: string (base URL path)
  - organizationName: string (GitHub organization/user name)
  - projectName: string (GitHub repository name)
  - themeConfig: object (theme-specific configuration)
- **Relationships**: Configures the entire documentation site
- **Validation rules**: Must be valid Docusaurus configuration format

### Navigation Structure
- **Name**: Sidebar Configuration
- **Fields**:
  - moduleTitle: string (title of the module)
  - chapters: list of chapter references
  - position: integer (order in navigation)
- **Relationships**: Links to chapter content files
- **Validation rules**: Must follow Docusaurus sidebar format

## State Transitions

### Content Development Workflow
1. **Draft**: Initial content creation
2. **Review**: Content review and technical accuracy verification
3. **Published**: Final content ready for deployment

## Relationships

- Module 2 contains 3 chapters
- Each chapter has associated code examples
- Navigation structure references all chapters
- Site configuration applies to all content