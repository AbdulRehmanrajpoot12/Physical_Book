# Data Model: The AI-Robot Brain (NVIDIA Isaac™)

## Entities

### Chapter Content
- **Name**: NVIDIA Isaac Sim
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/concept examples
- **Relationships**: Part of Module 3
- **Validation rules**: Must include all required sections (objectives, content, summary)

### Chapter Content
- **Name**: Isaac ROS & VSLAM
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/concept examples
- **Relationships**: Part of Module 3
- **Validation rules**: Must include practical examples of Isaac ROS and VSLAM concepts

### Chapter Content
- **Name**: Nav2 for Humanoid Navigation
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code/concept examples
- **Relationships**: Part of Module 3
- **Validation rules**: Must include navigation concepts specific to humanoid robots

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

- Module 3 contains 3 chapters
- Each chapter has associated examples and concepts
- Navigation structure references all chapters
- Site configuration applies to all content