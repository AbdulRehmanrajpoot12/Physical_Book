# Data Model: Module 4 - Vision-Language-Action (VLA)

## Entities

### Voice Command
- **Name**: Voice Command
- **Fields**:
  - audioData: binary (raw audio input)
  - textContent: string (transcribed text from Whisper)
  - timestamp: datetime (when command was received)
  - confidence: float (Whisper transcription confidence score)
  - userId: string (identifier for the user who spoke)
- **Relationships**: Part of user session, generates Robot Intent
- **Validation rules**: Must have audioData or textContent, confidence between 0 and 1

### Robot Intent
- **Name**: Robot Intent
- **Fields**:
  - intentType: string (navigation, manipulation, perception, etc.)
  - parameters: object (specific parameters for the intent)
  - confidence: float (confidence in intent classification)
  - extractedFrom: Voice Command (reference to source)
- **Relationships**: Generated from Voice Command, triggers Action Sequence
- **Validation rules**: Must have valid intentType and appropriate parameters

### Action Sequence
- **Name**: Action Sequence
- **Fields**:
  - steps: array of objects (ordered list of actions)
  - priority: integer (execution priority)
  - estimatedDuration: float (estimated time in seconds)
  - generatedFrom: Robot Intent (reference to source)
- **Relationships**: Generated from Robot Intent, maps to ROS 2 Actions
- **Validation rules**: Must have at least one step, each step must be valid

### ROS 2 Action
- **Name**: ROS 2 Action
- **Fields**:
  - actionType: string (navigation, manipulation, perception, etc.)
  - goal: object (specific goal parameters)
  - feedbackTopic: string (topic for feedback)
  - resultTopic: string (topic for results)
  - status: string (pending, executing, succeeded, failed)
- **Relationships**: Mapped from Action Sequence, executed by robot
- **Validation rules**: Must have valid actionType and goal parameters

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

### Voice Command Processing Workflow
1. **Captured**: Audio input received
2. **Transcribed**: Speech converted to text using Whisper
3. **Classified**: Intent extracted from text
4. **Completed**: Ready for action planning

### Action Execution Workflow
1. **Planned**: Action sequence generated from intent
2. **Scheduled**: ROS 2 actions queued for execution
3. **Executing**: Actions being executed by robot
4. **Completed**: Execution finished (success or failure)

## Relationships

- Voice Command generates 1 Robot Intent
- Robot Intent generates 1 Action Sequence
- Action Sequence maps to multiple ROS 2 Actions
- Module 4 contains 3 chapters
- Each chapter has associated examples and concepts
- Navigation structure references all chapters
- Site configuration applies to all content