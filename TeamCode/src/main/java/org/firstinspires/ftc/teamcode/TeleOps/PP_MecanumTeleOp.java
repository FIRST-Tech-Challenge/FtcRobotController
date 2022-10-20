package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDs.PIDController;


@TeleOp(name = "PP_MecanumTeleOp")
public class PP_MecanumTeleOp extends OpMode {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring class members to be used in other methods
    private ElapsedTime runtime = new ElapsedTime();
    PIDController motorPID = new PIDController(0,0,0,runtime);

    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, slideMotorLeft, slideMotorRight, armMotor;
    private Servo clawJoint, wristJoint;
    // Claw Servo open and close constants
    final int OPEN = 0;
    final double CLOSE = 0.5; // probably adjust later

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
        slideMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("liftMotor");
        slideMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("liftMotor");
        armMotor = (DcMotorEx)hardwareMap.dcMotor.get("armMotor");


        // Declaring our servos
        clawJoint = hardwareMap.get(Servo.class, "clawJoint");
        wristJoint = hardwareMap.get(Servo.class, "wristJoint");


        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Running without an encoder allows us to plug in a raw value rather than one that is proportional
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // to the motors total power. Ex. motor.setPower(0.5); would equal 50% if you run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Running without an encoder does NOT disable counting
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        clawJoint.setPosition(OPEN);

        }// end of init

        @Override
        public void loop(){
        // Our variables
        boolean precisionToggle = gamepad1.right_trigger > 0.1;



        drive(precisionToggle);
        arm();
        }



//        BOT METHODS       \\
    public void drive(boolean precisionToggle){
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

        if (precisionToggle) {
            motorFrontLeft.setPower(frontLeftPower * 0.6);
            motorBackLeft.setPower(backLeftPower * 0.6);
            motorFrontRight.setPower(frontRightPower * 0.6);
            motorBackRight.setPower(backRightPower * 0.6);
        }

        else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

    } // end of drive()

    public void arm(){
        // BUTTONS
        if(gamepad2.a){
            goToJunction(1000);
            // 1000 represents an arbitrary value for the max, 700 mid, 400 low, 0 ground
        }
        else if(gamepad2.b){
            goToJunction(700);
        }
        else if(gamepad2.y){
            goToJunction(400);
        }
        else if(gamepad2.x){
            goToJunction(0);
        }

        // DPAD
        if(gamepad2.dpad_left){
            clawRotate(-0.01);
        }
        else if(gamepad2.dpad_right){
            clawRotate(0.01);
        }
        else if(gamepad2.dpad_up){
            armPivot(-500); // pivots up
        }
        else if(gamepad2.dpad_down){
            armPivot(500); // pivots down
        }

        // BUMPER
        if(gamepad2.right_bumper){
            claw();
        }

        // TRIGGERS
        if(gamepad2.right_trigger > 0.2){
            slides(1);
        }
        else if(gamepad2.left_trigger > 0.2){
            slides(-1);
        }
    }

    public void goToJunction(int target){
        double currentPosition = (slideMotorLeft.getCurrentPosition() + slideMotorRight.getCurrentPosition())/2.0; // average position

        slideMotorLeft.setTargetPosition(target);
        slideMotorRight.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            slideMotorLeft.setPower(motorPID.update(target, currentPosition));
            slideMotorRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideMotorLeft.getCurrentPosition() + slideMotorRight.getCurrentPosition())/2.0; // average new position
        }
    }


    public void armPivot(int target){
        double currentPosition = armMotor.getCurrentPosition(); // average position
        armMotor.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = armMotor.getCurrentPosition(); // average new position
        }
    }

    public void clawRotate(double increment){
        wristJoint.setPosition(wristJoint.getPosition() + increment);
    }

    public void claw(){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
    }

    public void slides(int increment){
        double currentPosition = (slideMotorLeft.getCurrentPosition() + slideMotorRight.getCurrentPosition())/2.0; // average position
        int target = (int)currentPosition + increment; // we want to set the target to just above the current position every time this loop runs

        slideMotorLeft.setTargetPosition(target + increment);
        slideMotorRight.setTargetPosition(target + increment);

        while(Math.abs(target - currentPosition) > 5) {
            slideMotorLeft.setPower(motorPID.update(target, currentPosition));
            slideMotorRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideMotorLeft.getCurrentPosition() + slideMotorRight.getCurrentPosition())/2.0; // average new position
        }


    }

   /*public void lift(boolean precisionToggle){
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

*/
   }// end of class
