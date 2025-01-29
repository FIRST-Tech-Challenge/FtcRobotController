package org.firstinspires.ftc.teamcode.Final.Final.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.mainEnum;

@Autonomous(name = "Into the Deep Autonomous", group = "Final")
public class IntoTheDeepAuto extends LinearOpMode implements AutoInterface {
    hardware hardware = new hardware();
    calculations calculations = new calculations();


    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes
        armInit();
        wheelInit();
        sensorInit();
        servoInit();

        // Direction
        setArmDirection();
        setWheelDirection();

        // Brakes
        armBrake();
        wheelBrake();

        // Telemetry
        //telemetry();
        calculations.timer.reset();


        waitForStart();
        while (opModeIsActive() && calculations.totalGameTime.seconds() <= 28) {
            locateBlock(calculations.startDistanceToBlock);
            break;
        }
    }


    // Initialization functions
    @Override
    public void armInit() {
        hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
        hardware.lift = hardwareMap.get(DcMotor.class, "lift");
        hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
    }

    @Override
    public void wheelInit() {
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void sensorInit() {
        //hardware.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        hardware.distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceSensorFront");
        hardware.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");
        hardware.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        hardware.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
    }

    @Override
    public void servoInit() {
        hardware.door = hardwareMap.get(Servo.class, "door");
        hardware.topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
        hardware.bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
    }

    // Direction setup
    @Override
    public void setArmDirection() {
        hardware.lift.setDirection(DcMotor.Direction.REVERSE);
        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
        hardware.hopper.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void setWheelDirection() {
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }


    // Braking functions
    @Override
    public void armBrake() {
        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void wheelBrake() {
        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void setWheelSpeed(mainEnum state, double speed) {
        switch (state) {
            case FORWARD:
                hardware.frontLeft.setPower(speed);
                hardware.frontRight.setPower(speed);
                hardware.backLeft.setPower(speed);
                hardware.backRight.setPower(speed);
                break;

            case BACKWARD:
                hardware.frontLeft.setPower(-speed);
                hardware.frontRight.setPower(-speed);
                hardware.backLeft.setPower(-speed);
                hardware.backRight.setPower(-speed);
                break;

            case STRAFE_LEFT:
                hardware.frontLeft.setPower(-speed);
                hardware.frontRight.setPower(speed);
                hardware.backLeft.setPower(speed);
                hardware.backRight.setPower(-speed);
                break;

            case STRAFE_RIGHT:
                hardware.frontLeft.setPower(speed);
                hardware.frontRight.setPower(-speed);
                hardware.backLeft.setPower(-speed);
                hardware.backRight.setPower(speed);
                break;

            case TURN_LEFT:
                hardware.frontLeft.setPower(-speed);
                hardware.frontRight.setPower(speed);
                hardware.backLeft.setPower(-speed);
                hardware.backRight.setPower(speed);
                break;

            case TURN_RIGHT:
                hardware.frontLeft.setPower(speed);
                hardware.frontRight.setPower(-speed);
                hardware.backLeft.setPower(speed);
                hardware.backRight.setPower(-speed);
                break;
            case STOP:
                hardware.frontLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.backRight.setPower(0);
                break;
            default:
                // Handle invalid direction case
                hardware.frontLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.backRight.setPower(0);
                break;
        }
    }

    @Override
    public void setArmSpeed(mainEnum state, double speed) {
        switch (state) {
            case LIFT:
                hardware.lift.setPower(speed); // Set lift motor power
                break;
            case MANTIS:
                hardware.mantis.setPower(speed); // Set mantis motor power
                break;
            case HOPPER:
                hardware.hopper.setPower(speed); // Set hopper motor power
                break;
            default:
                hardware.mantis.setPower(calculations.mantisHold);
                hardware.lift.setPower(calculations.liftHold);
                hardware.hopper.setPower(calculations.hopperHold);
                break;
        }
    }

    @Override
    public void setClawSpeed(mainEnum state, double speedTopGrabber, double speedBottomGrabber, double wristSpeed, int doorPos) {
        switch (state) {
            case GRABBER:
                hardware.topGrabber.setPower(-speedTopGrabber);
                hardware.bottomGrabber.setPower(speedBottomGrabber);
                break;
            case DOOR:
                hardware.door.setPosition(doorPos);
                break;
            default:
                hardware.topGrabber.setPower(calculations.grabberHold);
                hardware.bottomGrabber.setPower(calculations.grabberHold);
                break;
        }
    }

    // Miscellaneous
    @Override
    public void telemetry() {
        telemetry.addLine("Code is running");
        telemetry.update();
    }

    @Override
    public void whileMotorsBusy() {
        telemetry.addLine("Code is running");
    }

    public void startingPosition(double distanceToBackWall) {
        final double tolerance = 0.1;
        final double robotWidthInches = 16.75;
        final double fieldWidthInInches = 72;
        final double goalDistanceFromSideWall = (fieldWidthInInches - robotWidthInches) / 2;
        final double maxExecutionTimeSeconds = 30; // Timeout to avoid infinite loops

        // Move to target distance from the back wall
        long startTime = System.currentTimeMillis();
        while (true) {
            double currentDistance = hardware.distanceSensorBack.getDistance(DistanceUnit.INCH);

            // Break if within tolerance or timeout
            if (Math.abs(currentDistance - distanceToBackWall) <= tolerance ||
                    System.currentTimeMillis() - startTime > maxExecutionTimeSeconds * 1000) {
                setWheelSpeed(mainEnum.STOP, 0);
                break;
            }

            // Move forward or backward
            if (currentDistance < distanceToBackWall) {
                setWheelSpeed(mainEnum.FORWARD, 0.3);
            } else if (currentDistance > distanceToBackWall) {
                setWheelSpeed(mainEnum.BACKWARD, 0.3);
            }
        }

        // Align to the middle of the field
        startTime = System.currentTimeMillis();
        while (true) {
            double leftDistance = hardware.distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double rightDistance = hardware.distanceSensorRight.getDistance(DistanceUnit.INCH);

            // Break if within tolerance or timeout
            if (Math.abs(leftDistance - rightDistance - robotWidthInches) <= tolerance ||
                    System.currentTimeMillis() - startTime > maxExecutionTimeSeconds * 1000) {
                setWheelSpeed(mainEnum.STOP, 0);
                break;
            }

            // Adjust position
            if (leftDistance > rightDistance) {
                setWheelSpeed(mainEnum.STRAFE_LEFT, 0.3);
            } else if (rightDistance > leftDistance) {
                setWheelSpeed(mainEnum.STRAFE_RIGHT, 0.3);
            }
        }
    }


    public void locateBlock(double distanceFromBlock) {
        while (hardware.distanceSensorLeft.getDistance(DistanceUnit.INCH) >= distanceFromBlock) {
            setWheelSpeed(mainEnum.FORWARD, calculations.driveSpeed);
        }
//        calculations.timer.reset();
//        while (calculations.timer.seconds() <= calculations.timeToRotate360/4) {
//            setWheelSpeed(mainEnum.TURN_LEFT, calculations.turnSpeed);
//        }

        setWheelSpeed(mainEnum.STOP, 0);
    }

    public void executeTask(String action, double target, double speed) {
        calculations.timer.reset();
        switch (action) {
            case "STARTING_POS":
                startingPosition(target);
                break;
            case "LOCATE_BLOCK":
                locateBlock(target);
            case "FORWARD":
                while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) > target) {
                    setWheelSpeed(mainEnum.FORWARD, speed);
                }
                setWheelSpeed(mainEnum.FORWARD, 0);
                break;
            case "BACKWARD":
                while (hardware.distanceSensorBack.getDistance(DistanceUnit.INCH) <= target) {
                    setWheelSpeed(mainEnum.BACKWARD, speed);
                }
                break;
            case "TURN_RIGHT":
                calculations.timer.reset();
                while (calculations.timer.seconds() < target) {
                    setWheelSpeed(mainEnum.TURN_RIGHT, speed);
                }
                setWheelSpeed(mainEnum.TURN_RIGHT, 0);
                break;
            case "TURN_LEFT":
                while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) < target) {
                    setWheelSpeed(mainEnum.TURN_LEFT, speed);
                }
                break;
            case "STRAFE_RIGHT":
                while (hardware.distanceSensorRight.getDistance(DistanceUnit.INCH) < target) {
                    setWheelSpeed(mainEnum.STRAFE_RIGHT, speed);
                }
                break;
            case "STRAFE_LEFT":
                while (hardware.distanceSensorLeft.getDistance(DistanceUnit.INCH) < target) {
                    setWheelSpeed(mainEnum.STRAFE_LEFT, speed);
                }
                break;
            case "LIFT_ARM":
                while (calculations.timer.seconds() < target) {
                    setArmSpeed(mainEnum.MANTIS, speed);
                }
                break;
            case "LIFT_HOPPER":
                calculations.timer.reset();
                while (calculations.timer.seconds() < target) {
                    setArmSpeed(mainEnum.HOPPER, speed);
                }
                setArmSpeed(mainEnum.HOPPER, calculations.hopperHold);
                break;
            case "ROTATE_CLAWS":
                calculations.timer.reset();
                while (calculations.timer.seconds() < target) {
                    hardware.topGrabber.setPower(1);
                    hardware.bottomGrabber.setPower(-1);
                }
                hardware.topGrabber.setPower(0);
                hardware.bottomGrabber.setPower(0);
                break;
            case "DOOR_CONTROL":
                hardware.door.setPosition(target);
                break;
            case "STOP":
                setWheelSpeed(mainEnum.STOP, 0);
                break;
        }
    }

    public void toSample() {
        double startingDistanceFromBackWall = 5;
        // Add the action, target, and speed for each step
        String[][] taskSequence = {
                //Move 12 inches from the wall
                {"BACKWARD", Double.toString(startingDistanceFromBackWall), Double.toString(calculations.driveSpeed)},
                //Move thr robot forward until the left sensor detects a block
                //{"LOCATE_BLOCK", Double.toString(calculations.startDistanceToBlock), null},
                //If it is # inches away from the submersable, go backwards
                //If it is 2 inches from the wall, repeat but with left sensor

                //If a block is detected, turn left or right based on which sensor it used
                //TODO find if rotating 90 degrees gets the robot to face the block

                //Stop
                {"STOP", null, null}
        };

        // Execute each task
        for (String[] task : taskSequence) {
            executeTask(task[0], Double.parseDouble(task[1]), Double.parseDouble(task[2]));
        }
    }

    private void test(){
        executeTask("TURN_RIGHT", calculations.timeToRotate360, calculations.turnSpeed);
    }
}

