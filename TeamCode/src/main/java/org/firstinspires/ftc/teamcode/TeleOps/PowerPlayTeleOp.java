package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "PowerPlayTeleOp")
public class PowerPlayTeleOp extends OpMode {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring class members to be used in other methods
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor, liftMotor;
    private Servo armJoint, clawJoint, wristJoint;

    /**
     * Get the maximum absolute value from a static array of doubles
     * @param input the input array of double values
     * @return the maximum value from the input array
     */

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
    public void init() {
        // Declaring our motors
        motorFrontLeft = (DcMotorEx)hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx)hardwareMap.dcMotor.get("BR");
        intakeMotor = (DcMotorEx)hardwareMap.dcMotor.get("intakeMotor");
        liftMotor = (DcMotorEx)hardwareMap.dcMotor.get("liftMotor");

        // Declaring our servos
        armJoint = hardwareMap.get(Servo.class, "armJoint");
        clawJoint = hardwareMap.get(Servo.class, "clawJoint");
        wristJoint = hardwareMap.get(Servo.class, "wristJoint");


        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        }// end of init

        @Override
        public void loop(){
        boolean precisionToggle = gamepad1.b;
        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;

        drive();

        intake(rightBumperPressed, leftBumperPressed);  // left_bumper = extake | right_bumper = intake

        lift(precisionToggle); // left trigger = lower | right trigger = raise
        }



//        bot methods       \\

    public void drive(){
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the mecanum motor powers
        double frontLeftPower = (y + x + 2 * rx) / denominator;
        double backLeftPower = (y - x + 2 * rx) / denominator;
        double frontRightPower = (y - x - 2 * rx) / denominator;
        double backRightPower = (y + x - 2 * rx) / denominator;


        // Cube the motor powers
        frontLeftPower = Math.pow(frontLeftPower, 3);
        frontRightPower = Math.pow(frontRightPower, 3);
        backLeftPower = Math.pow(backLeftPower, 3);
        backRightPower = Math.pow(backRightPower, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        });

        // Resize the motor power values
        if (maxValue > 1) {
            frontLeftPower /= maxValue;
            frontRightPower /= maxValue;
            backLeftPower /= maxValue;
            backRightPower /= maxValue;
        }

        if (gamepad1.left_trigger > .5) {
            motorFrontLeft.setPower(frontLeftPower * 0.3);
            motorBackLeft.setPower(backLeftPower * 0.3);
            motorFrontRight.setPower(frontRightPower * 0.3);
            motorBackRight.setPower(backRightPower * 0.3);
        }

        else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

    } // end of drive()

   public void intake(boolean rightBumperPressed, boolean leftBumperPressed){
    if(rightBumperPressed){
       intakeMotor.setPower(1);
     }
     else if (leftBumperPressed){
        intakeMotor.setPower(-1);
     }
   }// end of intake()

   public void lift(boolean precisionToggle){
   if(gamepad1.right_trigger > 0){
        if(precisionToggle)
            liftMotor.setPower(gamepad1.right_trigger * 0.6);

        else
            liftMotor.setPower(gamepad1.right_trigger);
       }

   if(gamepad1.left_trigger > 0){
        if (precisionToggle)
            liftMotor.setPower(-(gamepad1.left_trigger * 0.6)); // making this value negative because we want the motor to spin in the opposite direction

        else
            liftMotor.setPower(-gamepad1.left_trigger);
       }
   }// end of lift()








}// end of class
