# Connections

## Control Hub

### Motors

| Motor Port | Name | Description       | Encoder |
|------------|------|-------------------|---------|
| 0          | FL   | Front Left motor  | -       |
| 1          | FR   | Front Right motor | -       |
| 2          | BL   | Back Left motor   | -       |
| 3          | BR   | Back Right motor  | -       |

### Servos

## Expansion Hub

### Motors

| Motor Port | Name           | Description                   | Encoder       |
|------------|----------------|-------------------------------|---------------|
| 0          | HangArmRight   | Motor for right hang arm      | -             |
| 1          | DeliveryPivot  | Motor for delivery pivot      | Pivot encoder |
| 2          | DeliverySlider | Motor for the delivery slider | Slide encoder |
| 3          | HangArmLeft    | Motor for left hang arm       | -             |

### Servos

| Servo Port | Name        | Description               |
|------------|-------------|---------------------------|
| 0          | Elbow       | Intake Elbow servo        |
| 1          | RightIntake | CR servo for right intake |
| 2          | LeftIntake  | CR servo for left intake  |
| 3          | -           | -                         |
| 4          | -           | -                         |
| 5          | -           | -                         |



## I2C

| I2C Bus | Name        | Description                                |
|---------|-------------|--------------------------------------------|
| 0       | -           | IMU(vert. deadwheel, horizontal deadwheel) |
| 1       | ColorSensor | Intake color sensor                        |

## Digital

| Port | Name  | Description  |
|------|-------|--------------|
| 0    | Red   | Feedback LED |
| 1    | Green | Feedback LED |
