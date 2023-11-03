package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.MovementMethod;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    boolean placingPixel = false;
    boolean searching;

    public void encoderDrive(double power, double inches, MOVEMENT_DIRECTION movement_direction) {


        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 19.2;
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
            rightBackDriveMotor.setPower(.97*power);

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
            leftBackDriveMotor.setPower(.97 * power);
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

    public void placePixel(){
        //multiply sensor value by gain
        float gain = 10;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value.
        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

//        // If possible, turn the light on in the beginning (it might already be on anyway,
//        // we just make sure it is if we can).
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            colorSensor1.setGain(gain);
            colorSensor2.setGain(gain);

            // Get the normalized colors from the sensor
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors1.toColor(), hsvValues1);
            Color.colorToHSV(colors2.toColor(), hsvValues2);

            if (colorSensor1 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
            }
            if (colorSensor2 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors1.red)
                    .addData("Green", "%.3f", colors1.green)
                    .addData("Blue", "%.3f", colors1.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues1[0])
                    .addData("Saturation", "%.3f", hsvValues1[1])
                    .addData("Value", "%.3f", hsvValues1[2]);
            telemetry.addData("Alpha", "%.3f", colors1.alpha);

            telemetry.addLine()
                    .addData("Red 2", "%.3f", colors2.red)
                    .addData("Green 2", "%.3f", colors2.green)
                    .addData("Blue 2", "%.3f", colors2.blue);
            telemetry.addLine()
                    .addData("Hue 2", "%.3f", hsvValues2[0])
                    .addData("Saturation 2", "%.3f", hsvValues2[1])
                    .addData("Value 2", "%.3f", hsvValues2[2]);
            telemetry.addData("Alpha 2", "%.3f", colors2.alpha);

            //red
            if (hsvValues1[2] > .07){
                telemetry.addData("Color 1 ", "blue");
            }
            //blue
            else if (hsvValues1[2] > .06) {
                telemetry.addData("Color 1 ", "red");
            }

            if (hsvValues2[2] > .07){
                telemetry.addData("Color 1 ", "blue");
            }
            //blue
            else if (hsvValues2[2] > .06) {
                telemetry.addData("Color 1 ", "red");
            }

            if (gamepad1.a){
                placingPixel = true;
            }

            while (placingPixel){
                searching = true;
                leftFrontDriveMotor.setPower(0.2);
                leftBackDriveMotor.setPower(0.2);
                rightFrontDriveMotor.setPower(0.2);
                rightBackDriveMotor.setPower(0.2);
                while (searching) {
                    if (gamepad1.b){
                        leftFrontDriveMotor.setPower(0);
                        leftBackDriveMotor.setPower(0);
                        rightFrontDriveMotor.setPower(0);
                        rightBackDriveMotor.setPower(0);
                        searching = false;
                        break;
                    }
                    if (hsvValues1[2] > 0.1) {
                        leftFrontDriveMotor.setPower(0);
                        leftBackDriveMotor.setPower(0);
                    }
                    if (hsvValues2[2] > 0.1) {
                        rightFrontDriveMotor.setPower(0);
                        rightBackDriveMotor.setPower(0);
                    }
                    if (hsvValues1[2] > .1 && hsvValues2[2] > .03) {
                        searching = false;
                    }
                }
                telemetry.addData("Status: ", "Done");
                placingPixel = false;
            }

            telemetry.update();


        }
    }

    public void declareHardwareProperties() {


        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontright");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontleft");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "backright");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "backleft");

        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
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

