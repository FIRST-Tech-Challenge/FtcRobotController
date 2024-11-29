# Connections

## CONTROL HUB

### Motors

| Motor Port | Name | Description       | Encoder |
|------------|------|-------------------|---------|
| 0          | FL   | Front Left motor  | -       |
| 1          | FR   | Front Right motor | -       |
| 2          | BL   | Back Left motor   | -       |
| 3          | BR   | Back Right motor  | -       |

### Servos

| Servo Port | Name            | Description               |
|------------|-----------------|---------------------------|
| 0          | Limelight Servo | Currently broken          |
| 1          | Elbow           | Intake Elbow servo        |
| 2          | RightIntake     | CR servo for right intake |
| 3          | LeftIntake      | CR servo for left intake  |
| 4          | Stopper         | Servo for stopper         |
| 5          | -               | -                         |

## I2C

| I2C | Name        | Description                     |
|-----|-------------|---------------------------------|
| 0   | odo         | Odometry computer               |
| 1   | -           | -                               |
| 2   | ColorSensor | Color sensor for rolling intake |
| 3   | -           | -                               |

## EXPANSION HUB

### Motors

| Motor Port | Name           | Description                   | Encoder       |
|------------|----------------|-------------------------------|---------------|
| 0          | HangArmRight   | Motor for right hang arm      | -             |
| 1          | DeliveryPivot  | Motor for delivery pivot      | Pivot encoder |
| 2          | DeliverySlider | Motor for the delivery slider | Slide encoder |
| 3          | HangArmLeft    | Motor for left hang arm       | -             |

### Servos

## I2C

## Digital

| Port | Name  | Description  |
|------|-------|--------------|
| 0    | Red   | Feedback LED |
| 1    | Green | Feedback LED |
