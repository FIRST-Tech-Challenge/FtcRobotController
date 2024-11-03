# Connections

## Motors

| Motor Port | Name | Description       | Encoder          |
|------------|------|-------------------|------------------|
| 0          | FL   | Front Left motor  | Straight encoder |
| 1          | BL   | Back Left motor   | Strafe encoder   |
| 2          | FR   | Front Right motor | -                |
| 3          | BR   | Back Right motor  | -                |

## Servos

### Control Hub

| Servo Port | Name        | Description                         |
|------------|-------------|-------------------------------------|
| 0          | ClawRight   | Right claw grabber                  |
| 1          | ClawMiddle  | Servo for the clas swivel           |
| 2          | ClawLeft    | Left claw grabber                   |
| 3          | SlideElbow  | 35 kg servo that moves the claw arm |
| 4          | DroneElbow  | To be repurposed as hold for drone  |
| 5          | DroneLaunch | Servo that launches the drone       |

### Expansion Hub

| Servo Port | Name        | Description                                                 |
|------------|-------------|-------------------------------------------------------------|
| 0          | CamServo    | Servo for the camera swivel                                 |
| 1          | IntakeElbow | Servo that controls the intake elbow swivel                 |
| 2          | IntakeAxel  | Continuous rotation servo that rotates to intake or outtake |

## I2C

| I2C Bus | Name       | Description                |
|---------|------------|----------------------------|
| 0       | IMU        | Reserved for IMU           |
| 1       | LeftColor  | Left Color Sensor on Claw  |
| 2       | RightColor | Right Color Sensor on Claw |