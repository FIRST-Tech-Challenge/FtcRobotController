## Sharyland Robotics FTC Repository

Welcome to the official multi-team repository for Sharyland Robotics, home to our six FTC teams.
This repository is structured for collaborative development across all teams while maintaining clean
separation and modularity for each codebase.

## Repository Structure

This project includes **8 Android Studio modules**, each serving a unique role:

| Module Name  | Purpose                                                                         |
|--------------|---------------------------------------------------------------------------------|
| `Team00000`  | Sample module used for modeling, experimentation, and mentor-created utilities. |
| `Team12395`  | Code for **Team 12395 - Rattler Robotics.**                                     |
| `Team12397`  | Code for **Team 12397 - High Voltage Rattlers.**                                |
| `Team13580`  | Code for **Team 13580 - Tech Serpents.**                                        |
| `Team13581`  | Code for **Team 13581 - Fang Gang.**                                            |
| `Team13588`  | Code for **Team 13588 - Rattlerbots.**                                          |
| `Team13590`  | Code for **Team 13588 - Viperstrike.**                                          |
| `TeamShared` | Shared utilities, constants, and helper classes used across multiple teams.     |

Each team is responsible for maintaining their own module. Shared resources that benefit multiple teams
(e.g., vision pipelines, hardware wrappers, telemetry systems) should be placed in the `TeamShared` module.

## Getting Started

Each module follows the FTC SDK conventions for creating and organizing OpModes. To get Started:

1. Navigate to your team's module (e.g., `Team12395`).
2. Open `java/org.firstinspires.ftc.team12395`.
3. Create newOpModes by copying and customizing existing samples, or writing your own from scratch.
4. Register new OpModes by updating the `@TeleOp` or `@Autonomous` annotation and removing `@Disabled`.

## Creating OpModes

To create your own OpMode:

1. Copy an example OpMode from `FtcRobotController/java/org.firstinspires.ftc.robotcontroller.external.samples`.
2. Paste it into your team's package (e.g., `org.firstinspires.ftc.team13590`'`).
3. Rename the class and customize it to fit your robot's functionality.
4. Update the annotations so it appears properly in the Driver Station.

Example:
```java
@TeleOp(name = "Robot Centric", group = "Competition")
public class RobotCentricOpMode extends LinearOpMode {
    // your code here
}
``` 

## Running the Correct Team Module

When building and deploying code, be sure to select your team’s module in the Android Studio Run/Debug
configuration dropdown.

## Team Collaboration Tips

- Keep your module focused on *your team’s* robot. Use `TeamShared` for reusable code across multiple teams.
- Pull regularly to stay in sync with updates.
- Use Git branches for large changes or testing before merging to `main`. 
- Make sure to test the build and update relevant paths and package names throughout the module.