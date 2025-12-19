# Data Model: Docusaurus Book Structure

## Book Entity
- **name**: String (e.g., "AI-Physical Book")
- **version**: String (semantic versioning)
- **description**: String (book overview)
- **authors**: Array<String> (author names)
- **license**: String (license type)

## Module Entity
- **id**: String (unique identifier, e.g., "module-1")
- **title**: String (display title, e.g., "ROS 2 Basics")
- **description**: String (brief description of module content)
- **chapters**: Array<Chapter> (ordered list of chapters in module)
- **order**: Number (sequence position among modules)

## Chapter Entity
- **id**: String (unique identifier, e.g., "ros2-basics")
- **title**: String (display title)
- **description**: String (brief chapter overview)
- **content**: String (Markdown content)
- **prerequisites**: Array<String> (knowledge needed before reading)
- **learning_objectives**: Array<String> (what reader will learn)
- **module_id**: String (reference to parent module)
- **order**: Number (sequence position within module)
- **metadata**: Object (tags, difficulty, estimated_reading_time)

## Navigation Entity
- **sidebar_id**: String (identifier for sidebar grouping)
- **items**: Array<NavigationItem> (ordered list of navigation elements)
- **category_labels**: Object (labels for grouping chapters)

## NavigationItem Entity
- **type**: Enum ("doc", "category", "link")
- **id**: String (reference to document or category)
- **label**: String (display text)
- **href**: String (external link, if type="link")
- **items**: Array<NavigationItem> (children, if type="category")
- **priority**: Number (ordering within parent)

## Content Standards
- **Heading Hierarchy**: h1 for document title, h2-h6 for sections
- **Frontmatter**: YAML metadata at document start
- **Code Blocks**: Language-specific highlighting with examples
- **Admonitions**: Notes, warnings, tips using Docusaurus directives
- **Links**: Relative paths between documents, absolute for external

## Validation Rules
- Module IDs must be unique across the book
- Chapter IDs must be unique within a module
- All navigation references must point to existing documents
- Frontmatter must include required fields (title, description)
- Content must be in valid Markdown format
- No broken internal links