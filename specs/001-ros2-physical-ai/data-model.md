# Data Model: ROS 2 for Physical AI & Humanoid Robotics

## Entities

### Chapter Content
- **Name**: ROS 2 Basics
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code examples
- **Relationships**: Part of Module 1
- **Validation rules**: Must include all required sections (objectives, content, summary)

### Chapter Content
- **Name**: Python Agents & rclpy
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of code examples
- **Relationships**: Part of Module 1
- **Validation rules**: Must include practical examples of rclpy usage

### Chapter Content
- **Name**: URDF for Humanoids
- **Fields**:
  - title: string (chapter title)
  - content: string (main chapter content in Markdown)
  - objectives: list of strings (learning objectives)
  - summary: string (chapter summary)
  - examples: list of URDF examples
- **Relationships**: Part of Module 1
- **Validation rules**: Must include humanoid-specific URDF examples

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

- Module 1 contains 3 chapters
- Each chapter has associated code examples
- Navigation structure references all chapters
- Site configuration applies to all content