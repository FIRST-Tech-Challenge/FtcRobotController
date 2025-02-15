package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "drivetrainAutoBasketProgram", group = "Drive")
public class drivetrainAutoBasketProgram extends LinearOpMode {

    private CRServo intakeServoLeft, intakeServoRight;
    private TouchSensor intakeTouchSensor;
    private Servo intakePivotServo, basketServo;
    private DcMotorEx leftLiftMotor, rightLiftMotor;
    private Drivetrain drivetrain;
    public List<Double> blockPositions;

    @Override
    public void runOpMode() {
        // Initialize drivetrain
        drivetrain = new Drivetrain(hardwareMap, 3.5, 3, 12, 10, 1600);

        // Initialize lift motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // basket servo
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // intake servos
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
        intakeTouchSensor = hardwareMap.get(TouchSensor.class, "intakeTouchSensor");

        // initialize intake pivot servo to 0
        intakePivotServo.setPosition(0.0);

        blockPositions = new ArrayList<Double>();
        blockPositions.add(40.0);
        blockPositions.add(14.0);

        // output sensor data during init
        while (!isStarted()){
            telemetry.addData("horz: ", drivetrain.horizontalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("vert: ", drivetrain.verticalDistanceSensor.getDistance(DistanceUnit.CM));

            YawPitchRollAngles orientation = drivetrain.imu.getRobotYawPitchRollAngles();
            telemetry.addData("ang: ", orientation.getYaw(AngleUnit.DEGREES));

            telemetry.update();
        }

//        // at 28 seconds lift will drop to 0 just in case :)
//        new java.util.Timer().schedule(
//                new java.util.TimerTask() {
//                    @Override
//                    public void run() {
//                        if (rightLiftMotor.getCurrentPosition() > 20) {
//                            setLiftPosition(0);
//
//                            while (rightLiftMotor.getCurrentPosition() > 20){
//                                sleep(1);
//                            }
//
//                            stop();
//                        }
//                    }
//                },
//                28000);

        scoreInHighBasket(); // score the preloaded

        // loop through all block positions, first doing basket and then pickup
        for (double bPosition : blockPositions) {

            // its time to pick up a block, start by getting the robot in position
            while (opModeIsActive()) {
                drivetrain.alignToWall(Drivetrain.WallType.BACK, 40);
                drivetrain.alignToWall(Drivetrain.WallType.LEFT, bPosition);
                drivetrain.update();

                if (drivetrain.isAtTarget) {
                    drivetrain.stop();
                    break;
                }
            }

            // start intake
            runIntakeMotors(IntakeMode.NORMAL);
            intakePivotServo.setPosition(1.0);

            // now move forward while staying aligned to the left wall until intake is full
            while (opModeIsActive()) {

                drivetrain.alignToWall(Drivetrain.WallType.LEFT, bPosition);
                drivetrain.nudgeInDirection(0.0, 0.2); // move forward slowly
                drivetrain.update();

                if (intakeTouchSensor.isPressed()) {
                    drivetrain.stop();
                    runIntakeMotors(IntakeMode.STOP);

                    break;
                }
            }

            // deposit block in robot basket ---------------------------------
            intakePivotServo.setPosition(0.0); // pivot back to face robot basket

            // start un-intaking? block after 0.3 seconds, async call
            new java.util.Timer().schedule(
                    new java.util.TimerTask() {
                        @Override
                        public void run() {
                            runIntakeMotors(IntakeMode.REVERSE);
                        }
                    },
                    300);

            // stop unloading block after 2 seconds, async call
            new java.util.Timer().schedule(
                    new java.util.TimerTask() {
                        @Override
                        public void run() {
                            runIntakeMotors(IntakeMode.STOP);
                        }
                    },
                    2000);

            scoreInHighBasket(); // score block in high basket, code will continue after completion
        }
    }

    private enum IntakeMode {
        NORMAL,
        REVERSE,
        STOP
    }

    private void runIntakeMotors(IntakeMode mode) {
        switch (mode) {
            case NORMAL:
                intakeServoLeft.setPower(-1.0);
                intakeServoRight.setPower(1.0);
                break;
            case REVERSE:
                intakeServoLeft.setPower(1.0);
                intakeServoRight.setPower(-1.0);
                break;
            case STOP:
                intakeServoLeft.setPower(0.0);
                intakeServoRight.setPower(0.0);
                break;
        }
    }

    /**
     * Sets the lift to a specific position
     * @param targetPosition: target position in encoder ticks
     */
    private void setLiftPosition(int targetPosition) {
        leftLiftMotor.setTargetPosition(-targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setVelocity(2000);
        rightLiftMotor.setVelocity(2000);
    }

    /**
     * Move over to basket area, runs the lift up to the high basket, scores a block, then lowers lift back down
     * This is a blocking method.
     */
    private void scoreInHighBasket() {

        // stick to the back wall at a 35cm distance to align for basket
        while (opModeIsActive()) {

            drivetrain.alignToWall(Drivetrain.WallType.BACK, 35);
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 35);
            drivetrain.update();

            if (drivetrain.isAtTarget) {
                drivetrain.stop();

                break;
            }
        }

        // rotate to face the basket
        while (opModeIsActive()) {
            drivetrain.setAngle(-45);
            drivetrain.update();

            if (drivetrain.isAtTarget) {
                drivetrain.stop();

                break;
            }
        }

        setLiftPosition(3266); // high basket encoder position for lift

        // wait until lift is at target position
        while (rightLiftMotor.getCurrentPosition() < 2800){
            sleep(1);
        }

        // score block
        basketServo.setPosition(1.0); // up
        sleep(1000); // sleep 1 seconds
        basketServo.setPosition(0.0); // back down

        // return lift to default down position
        setLiftPosition(0);

        // wait until lift is at partially down to continue code execution
        while (rightLiftMotor.getCurrentPosition() > 1800){
            sleep(1);
        }

        // turn back
        while (opModeIsActive()) {
            drivetrain.setAngle(0);
            drivetrain.update();

            if (drivetrain.isAtTarget) {
                drivetrain.stop();

                break;
            }

        }
    }
}