# Team Code Guidelines & Class Descriptions

This document provides an overview of our codebase architecture by describing the roles and interactions of key classes, along with our naming conventions, file organization, and best practices. It is designed to help team members understand how each part of the system works independently as well as how they integrate into the overall project.

---

## Table of Contents

1. [Class Descriptions](#class-descriptions)
   - [Robot Class](#robot-class)
   - [RobotContainer Class](#robotcontainer-class)
   - [Subsystem Classes](#subsystem-classes)
   - [Command Classes](#command-classes)
   - [Constants Class](#constants-class)
2. [Naming Conventions](#naming-conventions)
3. [File & Package Organization](#file--package-organization)
4. [Code Structure & Design Patterns](#code-structure--design-patterns)
5. [Documentation & Comments](#documentation--comments)
6. [General Best Practices](#general-best-practices)
7. [Version Control & GitHub Best Practices](#version-control--github-best-practices)
8. [Additional Examples & Tips](#additional-examples--tips)

---

## Class Descriptions

### Robot Class

**Purpose:**  
The `Robot` class is the entry point of our application. It extends WPILib’s `TimedRobot` and is responsible for handling different robot modes such as initialization, autonomous, teleoperated, disabled, and test modes.

**Key Responsibilities:**
- **Initialization:**  
  Sets up the `RobotContainer`, which further configures subsystems, commands, and operator interfaces.
- **Periodic Execution:**  
  Calls the `CommandScheduler` to run subsystem periodic updates and command scheduling.
- **Mode Handling:**  
  Implements mode-specific methods (`robotInit()`, `robotPeriodic()`, `autonomousInit()`, `teleopInit()`, etc.) to control the robot’s behavior based on its state.
- **Diagnostics:**  
  Monitors sensor readings (e.g., battery voltage) and updates dashboards for real-time feedback.

---

### RobotContainer Class

**Purpose:**  
The `RobotContainer` class acts as the central hub for configuring the robot’s subsystems, commands, and operator inputs.

**Key Responsibilities:**
1. **Declarations at the Top:**  
   - **Controller Declarations:**  
     Declares and configures controller instances (e.g., `baseDriver` and `armDriver`).
   - **Subsystem Declarations:**  
     Instantiates all the subsystems (e.g., `s_Swerve`, `Wrist`, `Elbow`, `Elevator`, etc.) with clear prefixes to denote their roles.
   - **Command Declarations:**  
     Pre-declares commands (such as `ClimberUp` and `ClimberDown`) and any auxiliary components (e.g., autonomous chooser, global flags).

2. **Constructor:**  
   - **Subsystem & Command Initialization:**  
     Sets default commands (e.g., setting a default command for `Swerve`) and instantiates commands linked to subsystems.
   - **Autonomous Setup:**  
     Configures an autonomous command chooser that is pushed to the SmartDashboard.
   - **Input Configuration:**  
     Calls `configureButtonBindings()` to map controller buttons to commands, ensuring operator inputs trigger the correct actions.

3. **Configuration Methods:**  
   - **configureButtonBindings():**  
     Centralizes the mapping between controller buttons and command executions. It includes examples such as:
     - Binding a button to zero the robot’s heading.
     - Mapping triggers to control the climber.
     - Associating specific buttons with position-setting commands for different robot configurations.

---

### Subsystem Classes

**Purpose:**  
Subsystems represent the various hardware components of the robot (e.g., drivetrain, elevator, wrist, etc.). Each subsystem extends a base class (like `SubsystemBase`) and encapsulates the logic for interacting with its associated hardware.

**Key Responsibilities:**
- **Hardware Configuration:**  
  Initializes hardware components such as motors, sensors, and encoders.
- **Public Control Methods:**  
  Provides methods (e.g., `setSpeed()`, `setRotation()`, `stop()`) that are used by commands to manipulate hardware.
- **Periodic Updates:**  
  Implements the `periodic()` method to regularly update sensor readings or post diagnostics to dashboards.
- **Feedback and Setpoint Checking:**  
  Includes methods like `atSetpoint()` to determine if the subsystem has reached a desired state.

---

### Command Classes

**Purpose:**  
Commands encapsulate actions that the robot can perform, such as moving an arm to a set position or driving the robot. They interact with subsystems through well-defined interfaces.

**Key Responsibilities:**
- **Lifecycle Methods:**  
  Implement key methods (`initialize()`, `execute()`, `end()`, `isFinished()`) that define the command's behavior.
- **Subsystem Interaction:**  
  Call public methods on subsystems to perform actions. For example, the `SetPosition` command calls methods on the elevator, elbow, and wrist subsystems.
- **Dependency Management:**  
  Specify subsystem requirements to ensure that only one command controls a subsystem at a time.
- **Completion Criteria:**  
  Use conditions in `isFinished()` to determine when the command should end (e.g., when all subsystems have reached their setpoints).

---

### Constants Class

**Purpose:**  
The `Constants` class serves as a centralized repository for all fixed values used throughout the codebase, such as motor IDs, sensor configurations, PID coefficients, and tuning parameters.

**Key Responsibilities:**
- **Organized Grouping:**  
  Constants are organized into nested classes (e.g., `ControlConstants`, `Swerve`, `NotificationConstants`) to keep related values together.
- **Ease of Tuning:**  
  Parameters that require periodic adjustments are clearly marked with comments (e.g., `// TODO tune`).
- **Global Access:**  
  Allows easy access to configuration values across all subsystems and commands, promoting consistency and simplifying future modifications.

---

## Naming Conventions

- **Packages & Modules:**  
  - Use lowercase for package names.  
    _Examples:_ `frc.robot`, `frc.robot.commands`, `frc.robot.subsystems`
  
- **Classes:**  
  - Use **PascalCase** for class names.  
    _Examples:_ `Robot`, `RobotContainer`, `TeleopSwerve`, `SetPosition`, `Wrist`
  
- **Methods & Variables:**  
  - Use **camelCase** for methods and variable names.  
    _Examples:_ `robotInit()`, `configureButtonBindings()`, `batteryLowStartTime`

- **Constants:**  
  - Store all constants in a dedicated `Constants` class with nested static classes.
  - Use descriptive names in **camelCase** with inline comments if tuning is required.
  - _Examples:_ `Constants.ControlConstants.baseDriverControllerPort`, `Constants.Swerve.trackWidth`

---

## File & Package Organization

- **Modularization:**  
  - Divide code into logical packages based on functionality.  
  - Keep separate files for each subsystem, command, and utility function.

- **Main Entry Point:**  
  - Centralize primary logic in a main class (e.g., `Robot`) that handles mode transitions and overall system initialization.

- **Dedicated Containers:**  
  - Use `RobotContainer` to manage subsystems, commands, and operator interfaces, thereby decoupling hardware configuration from logic.

- **Subsystems & Commands Organization:**  
  - Place subsystem classes in a dedicated package (e.g., `frc.robot.subsystems`).
  - Place command classes in a separate package (e.g., `frc.robot.commands`), further organizing them by functionality (e.g., `Positioning`, `Swerve`).

---

## Code Structure & Design Patterns

- **Separation of Concerns:**  
  - Keep hardware control (subsystems) separate from decision-making and control logic (commands).

- **Command-Based Architecture:**  
  - Use the command-based framework to encapsulate discrete actions, making the code modular and reusable.
  - Ensure that each command has clear lifecycle methods (`initialize()`, `execute()`, `end()`, and `isFinished()`).

- **Default Commands & Bindings:**  
  - Assign default commands to subsystems to define their routine behavior.
  - Map operator inputs (buttons, triggers) to commands to keep the control scheme intuitive.

- **Centralized Scheduling:**  
  - Use a scheduler (such as WPILib’s `CommandScheduler`) to manage the lifecycle of commands and subsystem updates.

---

## Documentation & Comments

- **Inline Documentation:**  
  - Include concise comments that explain the rationale behind complex logic or design decisions.
  - Avoid over-commenting straightforward code.

- **Javadoc Comments:**  
  - Use Javadoc-style comments for public classes and methods to facilitate automated documentation generation.

- **Code Examples:**  
  - Provide usage examples within the documentation to help new team members understand how to use the classes and methods.

---

## General Best Practices

- **Consistency:**  
  - Maintain a uniform coding style across the entire codebase to minimize cognitive load.
  
- **Modularity:**  
  - Design components to be as independent as possible to simplify testing and reuse.

- **Error Handling:**  
  - Implement robust error handling and document assumptions or edge cases.

- **Refactoring:**  
  - Regularly refactor code to improve clarity and reduce complexity. Look out for duplicate code or methods that do too much.

- **Testing:**  
  - Use diagnostic tools (like SmartDashboard) for real-time monitoring of system performance.

---

## Version Control & GitHub Best Practices

- **Git & GitHub Usage:**  
  - Our team uses Git for version control and maintains our code repository on GitHub at [https://github.com/t3team-laptop](https://github.com/t3team-laptop).
  
- **Commit Often:**  
  - Make frequent commits with clear, descriptive messages.
  
- **Branching:**  
  - Develop new features or fixes in dedicated branches (e.g., `feature/add-autonomous-mode` or `bugfix/joystick-deadzone`) to keep the main branch stable.
  
- **Pull Requests & Code Reviews:**  
  - Create pull requests for merging changes and ensure peer review before merging.
  
- **Push Regularly:**  
  - Push commits to the remote repository often to back up work and keep the team updated.
  
- **Resolve Conflicts Promptly:**  
  - Address merge conflicts quickly and communicate with the team to maintain workflow efficiency.

- **Repository Documentation:**  
  - Keep the README and other repository documentation up-to-date for new team members and ongoing project clarity.

---

## Additional Examples & Tips

### Good vs. Bad Commit Messages

**Bad Commit Message Examples:**
- `Fixed stuff`
- `Changes`
- `Update code`

These messages are too vague and do not provide context or detail.

**Good Commit Message Examples:**
- `Fix joystick deadzone issue in TeleopSwerve command`
- `Refactor SetPosition command to improve subsystem dependency handling`
- `Update PID constants for wrist control based on recent tuning`
- `Add error handling for sensor initialization in Elevator subsystem`

**Commit Message Best Practices:**
- **Be Specific:** Clearly state what was changed and why.
- **Use the Imperative Mood:** For example, “Fix bug” instead of “Fixed bug.”
- **Include Context:** Reference tickets or issues if applicable.
- **Keep It Concise:** Use a short title (ideally 50 characters or less) with additional details in the body if necessary.

### Other Examples for Collaboration

- **Code Reviews:**  
  Request a review from a teammate before merging.  
  _Example comment:_ "I’ve refactored the SetPosition command to improve readability and error handling. Could you review the changes to ensure nothing breaks the existing behavior?"

- **Descriptive Branch Names:**  
  Use clear branch names, for instance:  
  - `feature/add-autonomous-mode`
  - `bugfix/joystick-deadzone`

- **Regular Communication:**  
  Inform the team about major changes through established channels (e.g., Slack or Discord) to keep everyone aligned.

- **Documentation Updates:**  
  Always update the relevant documentation when introducing new features or changes. For example, update the README with new setup instructions when the autonomous routine is modified.

---