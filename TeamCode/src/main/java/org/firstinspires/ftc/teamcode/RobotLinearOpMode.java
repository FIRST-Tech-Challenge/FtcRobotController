package org.firstinspires.ftc.teamcode;

import android.text.method.MovementMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file is where each method should be written or referenced. Then when writing Autos and TeleOps,
 * programmers need to make files that extends RobotLinearOpMode instead of extending LinearOpMode.
 */

@Autonomous(name="RobotLinearOpMode", group="Linear Opmode")
@Disabled
public abstract class RobotLinearOpMode extends LinearOpMode {

    // Construction //
    public RobotLinearOpMode() {

    }

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;



    public void encoderDrive(double power, double inches, MOVEMENT_DIRECTION movement_direction) {


        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 20;
        final double COUNTS_PER_ROTATION_AT_MOTOR = 28;
        final double TICKS_PER_ROTATION = (GEAR_RATIO * COUNTS_PER_ROTATION_AT_MOTOR);
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);


        }

        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }
            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget * 2);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget * 2);
            leftBackDriveMotor.setTargetPosition(leftBackTarget * 2);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget * 2);


            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget * 2);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget * 2);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget * 2);
            rightBackDriveMotor.setTargetPosition(rightBackTarget * 2);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kills the motors to prepare for next call of method
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    /*public void encoderLift(double power, double inches, LIFT_DIRECTION lift_direction) {

        //Specifications of hardware
        final double wheelDiameter = 1.5;
        final double wheelCircumference = (wheelDiameter * 3.141592653589793);
        final double ticksPerRotation = 28;
        final double ticksPerInch = (ticksPerRotation / wheelCircumference);

        int liftTarget;


        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTarget = leftLifter.getCurrentPosition() + (int) (inches * ticksPerInch);


        if (lift_direction == LIFT_DIRECTION.UP) {
            leftLifter.setTargetPosition(liftTarget);
            rightLifter.setTargetPosition(liftTarget);

            leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftLifter.setPower(power);
            rightLifter.setPower(power);


            while (leftLifter.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftLifter.setPower(0);
            rightLifter.setPower(0);

        }
    }*/

    /*public void encoderTurn(double power, double degrees, TURN_DIRECTION turn_direction) {





        //Declaration of important variables
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double DRIVE_GEAR_REDUCTION = 1.0;
        final double TICKS_PER_ROTATION = 150;
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_INCHES);
        //Measure in inches
        final double ROBOT_LENGTH = 14;
        final double ROBOT_WIDTH = 12;
        //Uses pythagorean theorem to find the radius from the center of the robot to a point on its turning circle and subsequently the circumference of said circle
        final double ROBOT_DIAMETER = (Math.sqrt(Math.pow(ROBOT_LENGTH, 2) + Math.pow(ROBOT_WIDTH, 2)));
        final double ROBOT_CIRCUMFERENCE = (ROBOT_DIAMETER * 3.141592653589793);
        //Finds the number of degrees each tick covers
        final double INCHES_PER_DEGREE = (ROBOT_CIRCUMFERENCE/360);
        final double TICKS_PER_DEGREE = (TICKS_PER_INCH * INCHES_PER_DEGREE);

        declareHardwareProperties();

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the taret # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (TICKS_PER_DEGREE);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (TICKS_PER_DEGREE);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (TICKS_PER_DEGREE);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (TICKS_PER_DEGREE);

        if(turn_direction == TURN_DIRECTION.TURN_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);
        }

        if(turn_direction == TURN_DIRECTION.TURN_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget);
            rightBackDriveMotor.setTargetPosition(rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);
        }

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
*/

    public void declareHardwareProperties() {


        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontleft");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "backleft");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "frontright");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "backright");

        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    enum MOVEMENT_DIRECTION {
        STRAFE_LEFT,
        STRAFE_RIGHT,
        FORWARD,
        REVERSE,
    }

    enum TURN_DIRECTION {
        TURN_LEFT,
        TURN_RIGHT

    }

    enum LIFT_DIRECTION {
        DOWN,
        UP
    }

}

