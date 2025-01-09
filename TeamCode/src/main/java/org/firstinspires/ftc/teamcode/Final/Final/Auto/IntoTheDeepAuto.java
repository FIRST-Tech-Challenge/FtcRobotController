package org.firstinspires.ftc.teamcode.Final.Final.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
        while (opModeIsActive()) {
            test();
        }
    }

    //Sets the pieces
    //@Override
    public void setSample() {
        //Goes up to the cage until its half an inch away
        while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) < calculations.cageDistance) {
            setWheelSpeed(mainEnum.FORWARD, calculations.driveSpeed);
        }
        //Strafes left until it doesn't detect the cage anymore
        while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) < calculations.cageDistance) {
            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
        }
        //Strafes left until the right side clears the cage
        while (calculations.timer.seconds() < calculations.clearCage) {
            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
        }
        //Turns right 90 degrees
        while (calculations.timer.seconds() < calculations.timeToRotate360) {
            setWheelSpeed(mainEnum.TURN_RIGHT, calculations.turnSpeed);
        }
        //Strafes left until it detects cage
        while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) > calculations.cageDistance) {
            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
        }
        //Strafes left for a few seconds
        while (calculations.timer.seconds() < calculations.smidge) {
            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
        }
        //Goes back to make room for arm
        while (calculations.timer.seconds() < calculations.makeSpaceForArm) {
            setWheelSpeed(mainEnum.BACKWARD, calculations.driveSpeed);
        }
        //Lifts up arm
        while (calculations.timer.seconds() < calculations.timeToLiftArm) {
            setArmSpeed(mainEnum.MANTIS, calculations.driveSpeed);
        }
        //Goes forward and lifts arm down
        while (calculations.timer.seconds() < calculations.cageDistance) {
            setWheelSpeed(mainEnum.FORWARD, calculations.driveSpeed);
        }
        while (calculations.timer.seconds() < calculations.timeToLiftArm) {
            setArmSpeed(mainEnum.MANTIS, -calculations.driveSpeed);
        }
        //Moves backwards to release sample
        while (calculations.timer.seconds() < calculations.smidge) {
            setWheelSpeed(mainEnum.BACKWARD, calculations.driveSpeed);
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


//    private void colorTemeletry() {
//        //int red = hardware.colorSensor.red();
//        //int blue = hardware.colorSensor.blue();
//        //int green = hardware.colorSensor.green();
//
//        telemetry.addData("Red", red);
//        telemetry.addData("Blue", blue);
//        telemetry.addData("Green", green);
//        telemetry.update();
//    }

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

    //@Override
    public void executeTask(String action, double target, double speed) {
        calculations.timer.reset(); // Reset the timer for timed actions
        switch (action) {
            case "MOVE_TO_DISTANCE":
                while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) > target) {
                    setWheelSpeed(mainEnum.FORWARD, speed);
                }
                setWheelSpeed(mainEnum.FORWARD, 0);
                break;
            case "STRAFE_TO_DISTANCE":
                while (hardware.distanceSensorFront.getDistance(DistanceUnit.INCH) < target) {
                    setWheelSpeed(mainEnum.STRAFE_LEFT, speed);
                }
                break;
            case "STRAFE_UNTIL_CLEAR":
                while (calculations.timer.seconds() < target) {
                    setWheelSpeed(mainEnum.STRAFE_LEFT, speed);
                }
                break;
            case "ROTATE":
                while (calculations.timer.seconds() < target) {
                    setWheelSpeed(mainEnum.TURN_RIGHT, speed);
                }
                break;
            case "MOVE_BACKWARD":
                while (calculations.timer.seconds() < target) {
                    setWheelSpeed(mainEnum.BACKWARD, speed);
                }
                break;
            case "LIFT_ARM":
                while (calculations.timer.seconds() < target) {
                    setArmSpeed(mainEnum.MANTIS, speed);
                }
                break;
            default:
                setWheelSpeed(mainEnum.FORWARD, 0);
                break;
        }
    }

    public void toSample() {
        // Define the task sequence
        String[][] taskSequence = {
                //Move the robot to be 5 inches from the back wall and then center middle of robot to be in the middle horizontally
                //Rotate 90 degrees left
                //Strafe until block is detected
                //Move forward until block is 19 inches away
                //Lower arm down
                //Engage claw
                //Move arm up
                //Engage claw
                //Move forward until wall is 4 inches away
                //Rotate right until wall is detected again
                //Move forward 3 inches
                //Lift the hopper all the away
                //Open door

        };

        // Execute each task
        for (String[] task : taskSequence) {
            executeTask(task[0], Double.parseDouble(task[1]), Double.parseDouble(task[2]));
        }
    }

    private void test(){
        //ROTATION
        while(calculations.timer.seconds() <= calculations.timeToRotate360){
            setWheelSpeed(mainEnum.TURN_RIGHT, calculations.turnSpeed);
        }
        setWheelSpeed(mainEnum.FORWARD, 0);
        //ARM

        //CLAW
    }
}

