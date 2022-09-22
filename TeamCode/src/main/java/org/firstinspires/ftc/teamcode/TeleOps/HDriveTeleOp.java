package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "HDriveTeleOp")
public class HDriveTeleOp extends OpMode {
    DcMotorEx leftMotor, rightMotor, midMotor;
    boolean precisionToggle = false;

    // Method used to find the max of more than two numbers since Math.max() only takes in two arguments
    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    @Override
    public void init(){
        leftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightMotor");
        midMotor = (DcMotorEx) hardwareMap.dcMotor.get("midMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Assuming the bot's front is facing directly away from the driver
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        midMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        drive();


    }

    public void drive(){
        double left_X  = gamepad1.left_stick_x;
        double left_Y = -gamepad1.left_stick_y;
        double right_X = gamepad1.right_stick_x;

        /*\
        Denominator is the largest motor power (absolute value) or 1
        This ensures all the powers maintain the same ratio, but only when
        at least one is out of the range [-1, 1]
        \*/

        double denominator = Math.max(Math.abs(left_X) + Math.abs(left_Y) + Math.abs(right_X), 1);

        // Calculate and assign motor powers
        double rightPow = (left_Y - right_X) / denominator; // if the right_X is positive, the right motor should go slower
        double midPow = (left_X) / denominator;
        double leftPow = (left_Y + right_X) / denominator; // if the right_X is positive, the left motor should go faster

        // Cube the motor powers
        leftPow = Math.pow(leftPow, 3);
        rightPow = Math.pow(rightPow, 3);
        midPow = Math.pow(midPow, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{leftPow, rightPow,midPow,});

        // Resize the motor power values to the same ratio
        if (maxValue > 1) {
            leftPow /= maxValue;
            rightPow /= maxValue;
            midPow /= maxValue;
        }

        if (gamepad1.a) precisionToggle = !precisionToggle;

        if (precisionToggle){
             leftMotor.setPower(leftPow * 0.3);
             rightMotor.setPower(rightPow * 0.3);
             midMotor.setPower(midPow * 0.3);
         }

        midMotor.setPower(midPow);
        rightMotor.setPower(rightPow);
        leftMotor.setPower(leftPow);

    } // end of drive()

    /*
    public void intake(boolean rightBumperPressed, boolean leftBumperPressed){
        if(rightBumperPressed){
            intakeMotor.setPower(1);
        }
        else if (leftBumperPressed){
            intakeMotor.setPower(-1);
        }
    }// end of intake()
     */
}
