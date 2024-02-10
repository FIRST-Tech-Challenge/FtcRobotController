package org.firstinspires.ftc.teamcode.drivecode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class Mecanum extends LinearOpMode {


    private DcMotor armRotate;
    private DcMotor armBrace;
    private DcMotor armExt;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo linearGripper;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //Initialize Variables
        //Tracks rotation of the arm's motor
        int rotation;
        //Tracks the extension of the arm
        int ext;
        //X and Y values of stick inputs to compile drive outputs
        double Ry;
        double Rx;
        double Ly;
        double Lx;
        double a;
        double h;
        //Value for armLock
        boolean armLocked;
        boolean c = false;
        boolean d = false;

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        linearGripper = hardwareMap.get(Servo.class, "linearGripper");

        a = .05;

        //Sets variables to 0 on initialization
        rotation = 0;
        ext = 0;
        h = 0;
        armLocked = false;
        waitForStart();

        if (opModeIsActive()) {

            //Sets behaviors and modes for motors
            //ArmExtension and ArmRotate are set to brake when receiving zero power
            //Arm Extension is set to run using encoder outputs and inputs
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBrace.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armBrace.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearGripper.getPosition();

            while (opModeIsActive()) {

                //Assigns variables to inputs
                //Ext and Rotation are set to receive inputs from encoders
                ext = armExt.getCurrentPosition();
                rotation = armRotate.getCurrentPosition();
                h = linearGripper.getPosition();
                //Multipliers are applied to X and Y and they are tied to sticks on the game pads.
                Ly = gamepad1.left_stick_y;
                Rx = gamepad1.right_stick_x;
                Lx = gamepad1.left_stick_x;
                Ry = gamepad1.right_stick_y;


                if (gamepad1.right_trigger > 0.5) {

                    if (Ly > 0.2 || Ly < -0.2 || Rx > .02 || Rx < -.02) {
                        // Waiting for Left Joystick to Move & Scaling the Motor Value
                        frontLeftMotor.setPower((-Ly+Rx)*0.4);
                        frontRightMotor.setPower((Ly+Rx)*0.4);
                        backLeftMotor.setPower((-Ly+Rx)*0.4);
                        backRightMotor.setPower((Ly+Rx)*0.4);
                    } else if (gamepad1.left_bumper) {
                        frontLeftMotor.setPower(-0.4);
                        frontRightMotor.setPower(-0.4);
                        backLeftMotor.setPower(0.4);
                        backRightMotor.setPower(0.4);
                    } else if (gamepad1.right_bumper) {
                        frontLeftMotor.setPower(0.4);
                        frontRightMotor.setPower(0.4);
                        backLeftMotor.setPower(-0.4);
                        backRightMotor.setPower(-0.4);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                } else {// Trigger isn't pressed
                    if (Ly > 0.2 || Ly < -0.2 || Rx > 0.2 || Rx < -0.2) {
                        frontLeftMotor.setPower(-Ly+Rx);
                        frontRightMotor.setPower(Ly+Rx);
                        backLeftMotor.setPower(-Ly+Rx);
                        backRightMotor.setPower(Ly+Rx);
                    } else if (gamepad1.left_bumper) {
                        frontLeftMotor.setPower(-1);
                        frontRightMotor.setPower(-1);
                        backLeftMotor.setPower(1);
                        backRightMotor.setPower(1);
                    } else if (gamepad1.right_bumper) {
                        frontLeftMotor.setPower(1);
                        frontRightMotor.setPower(1);
                        backLeftMotor.setPower(-1);
                        backRightMotor.setPower(-1);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                }

                //Arm rotation controls
                //Rotates up when Right Bumper is pressed
                //Rotates down when Left Bumper is pressed
                //Otherwise power is set to 0 (BRAKE)

                //if(armLocked == false){

                if(armLocked == true){
                    armRotate.setPower(0.05);
                    armBrace.setPower(0.05);
                } else{
                    if (gamepad2.right_bumper) {
                        armRotate.setPower(0.3);
                        armBrace.setPower(0.3);
                    } else if (gamepad2.left_bumper) {
                        armRotate.setPower(-0.2);
                        armBrace.setPower(-0.2);
                    } else {
                        armRotate.setPower(0);
                        armBrace.setPower(0);
                    }
                }//end arm rotation inputs



                //Arm extension controls
                //Moves up when X is pressed
                //Moves down when B is pressed
                //Otherwise set power to 0 (BRAKE)
                //Limiters are applied if the motors position is less than 350° or greater than 2900°
                //If the position is less than 350°, the arm can only extend forward
                //If the position is greater than 2900°, the arm can only retract
                if (ext > 350 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext < 350 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext > 2900 && ext > 350) {
                    if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } if(gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5){
                    armExt.setPower(-1);
                    sleep(1000000);
                } //end arm extension inputs

                //Gripper
                if (gamepad2.b && h < 3.7) {
                    c = true;
                    d = false;
                    linearGripper.setPosition(a+=.02);
                } else if (gamepad2.a) {
                    c = false;
                    d = true;
                    a = .01;
                    linearGripper.setPosition(.01);
                } else if (c == true) {
                    linearGripper.setPosition(a);
                } else if (d == true) {
                    linearGripper.setPosition(.01);
                }


                //Telemetry for debugging
                telemetry.addData("Current Arm Extension", ext);
                telemetry.addData("Current Arm Rotation", armRotate.getCurrentPosition());
                telemetry.addData("Arm lift Power", armRotate.getPower());
                telemetry.addData("Back Right Wheel Power", backRightMotor.getPower());
                telemetry.addData("Back Left Wheel Power", backLeftMotor.getPower());
                telemetry.addData("Front Right Motor Power", frontRightMotor.getPower());
                telemetry.addData("Front Left Motor Power", frontLeftMotor.getPower());
                telemetry.addData("Arm Locked: ", armLocked);
                telemetry.addData("Servo Position", linearGripper.getPosition());
                telemetry.update();
            } //end while loop
        } //end if loop
    } //end run method
} //end class
//Written by dolphindasher324
