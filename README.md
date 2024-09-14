# Capitol Robotics

# FTC Team Code Master Repository

Welcome to our FTC (First Tech Challenge) Team's Code Master Repository! This repository contains the entire codebase for our robot, including autonomous, teleop, and hardware configuration code.

## Table of Contents
- [About the Project](#about-the-project)
- [Installation](#installation)
- [Usage](#usage)
- [Branching Workflow](#branching-workflow)
- [Contributing](#contributing)


## About the Project
This repository contains all the code we are using to control our robot for the current FTC season. Our code is written in Java and organized into different modules for autonomous mode, teleop control, and hardware configuration.

### Key Features
- **Autonomous Mode**: Pre-programmed movements for various challenges.
- **TeleOp Mode**: Manual control for driving and manipulating the robot during the competition.
- **Hardware Configuration**: Code to initialize motors, sensors, and other components.

## Installation
To set up the repository on your local machine:
1. Clone the repository using Git:
    ```bash
    https://github.com/CapitalRobotics/Janxs_Brain.git
    ```
2. Open the project in Android Studio, as it is built for the FTC SDK.

3. Connect your FTC robot controller device to your development machine.

## Usage
- **Autonomous**: Select an autonomous mode from the robot controller app during the autonomous phase of the match.
- **TeleOp**: Use the gamepad controllers during the driver-controlled phase.

## Branching Workflow

We use a branching strategy to ensure smooth collaboration and protect the main branch from unintended changes. Here's how we manage the workflow:

1. **Clone the Repository**: Team members clone the main repository:
    ```bash
    https://github.com/CapitalRobotics/Janxs_Brain.git
    ```

2. **Create a Feature Branch**: Instead of working directly on the main branch, create a new feature branch for your work:
    ```bash
    git checkout -b feature/your-feature-name
    ```

3. **Branch Protection**: The main branch is protected, so no direct commits can be made. All changes must be merged via pull requests (PRs) after code review.

4. **Submit a Pull Request**: Once your feature is complete, open a pull request to merge your feature branch into the main branch. Your code will go through a review process before being merged.

## Contributing
1. Clone the repository and create a feature branch.
2. Commit your changes to the feature branch.
3. Submit a pull request for review.
4. Once approved, your changes will be merged into the main branch.



