package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "drivetrainAutoBasketProgram", group = "Drive")
public class drivetrainAutoBasketProgram extends LinearOpMode {

    private CRServo intakeServoLeft, intakeServoRight;
    private TouchSensor intakeTouchSensor;
    private Servo intakePivotServo, basketServo;
    private DcMotorEx leftLiftMotor, rightLiftMotor;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        // Initialize drivetrain
        drivetrain = new Drivetrain(hardwareMap, 1, 3, 30, 20, 0.8);

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

        waitForStart();

        // stick to the back wall at a 40cm distance to align for basket
        while (opModeIsActive()) {

            drivetrain.alignToWall(Drivetrain.WallType.BACK, 40);
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 40);
            drivetrain.update();

            telemetry.addData("isAlignedToWall: ", drivetrain.isMoving);
            telemetry.addData("horz: ", drivetrain.horizontalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("vert: ", drivetrain.verticalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("xWeights: ", drivetrain.xWeights.toString());
            telemetry.addData("yWeights: ", drivetrain.yWeights.toString());
            telemetry.addData("rWeights: ", drivetrain.rWeights.toString());
            telemetry.update();

            if (!drivetrain.isMoving) {
                drivetrain.stop();

                break;
            }
        }

        // rotate to face the basket
        while (opModeIsActive()) {
            drivetrain.setAngle(-45);
            drivetrain.update();

            if (!drivetrain.isMoving) {
                drivetrain.stop();

                break;
            }
        }

        scoreInHighBasket(); // score block in basket, code will continue after completion

        // pause for 1 second
        sleep(1000);

        // its time to pick up a block, start by getting the robot in position
        while (opModeIsActive()) {
            drivetrain.alignToWall(Drivetrain.WallType.BACK, 40);
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 37);
            drivetrain.update();

            if (!drivetrain.isMoving) {
                drivetrain.stop();
                break;
            }
        }

        // start intake
        runIntakeMotors(IntakeMode.NORMAL);
        intakePivotServo.setPosition(1.0);

        // pause for 1 second
        sleep(1000);

        // now move forward while staying aligned to the left wall until intake is full
        while (opModeIsActive()) {
            // stick to the back wall at a 20cm distance
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 37);
            drivetrain.nudgeInDirection(0.0, 0.2);
            drivetrain.update();

            if (intakeTouchSensor.isPressed()) {
                drivetrain.stop();
                runIntakeMotors(IntakeMode.STOP);
                break;
            }
        }

        // deposit block in robot basket
        intakePivotServo.setPosition(0.0);
        sleep(1000); // sleep 1 second
        runIntakeMotors(IntakeMode.REVERSE);
        sleep(3000); // sleep 3 second
        runIntakeMotors(IntakeMode.STOP);

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
     * Runs the lift up to the high basket, scores a block, then goes back down to starting position
     */
    private void scoreInHighBasket() {

        setLiftPosition(3266); // high basket encoder position for lift

        // wait until lift is at target position
        while (rightLiftMotor.getCurrentPosition() < 2890){
            sleep(1);
        }

        // score block
        basketServo.setPosition(1.0); // up
        sleep(1500); // sleep 1.5 seconds
        basketServo.setPosition(0.0); // back down
        sleep(1500); // sleep 1.5 seconds

        // return lift to default position
        setLiftPosition(0);

        // wait until lift is at target position
        while (rightLiftMotor.getCurrentPosition() > 20){
            sleep(1);
        }
    }
}