package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class AllWheelDrive extends LinearOpMode {


    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo linearGripper;
    private DcMotor armExt;
    private DcMotor armBrace;
    private DcMotor armRotate;

    //This function ic executed when theis OpMode is selected from the Driver Station
    @Override

    public void runOpMode() {

        int rotation;
        int ext;
        boolean armLocked;
        // X and Y inputs from the Driver
        double x;
        double y;
        double v;
        double c;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        linearGripper = hardwareMap.get(Servo.class, "linearGripper");
        armExt = hardwareMap.get(DcMotor.class,"armExt");
        armBrace = hardwareMap.get(DcMotor.class, "armExt");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");

        rotation = 0;
        ext = 0;
        armLocked = false;

        waitForStart();

        if (opModeIsActive()) {

            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBrace.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armBrace.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {

                ext = armExt.getCurrentPosition();
                rotation = armRotate.getCurrentPosition();

                y = gamepad1.left_stick_y;
                v = gamepad1.left_stick_x;
                x = gamepad1.right_stick_x;
                c = gamepad1.left_stick_y;


                // Farward/Backward
                if (y > 0.2 || y < -0.2){
                    frontLeftMotor.setPower(-y);
                    frontRightMotor.setPower(y);
                    backLeftMotor.setPower(-y);
                    backRightMotor.setPower(y);
                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }

                //Left/Right
                if (x > 0.2 || x < -0.2){
                    frontLeftMotor.setPower(x);
                    frontRightMotor.setPower(x);
                    backLeftMotor.setPower(x);
                    backRightMotor.setPower(x);
                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }

                //Gripper
                if (gamepad1.b) {
                    linearGripper.setPosition(.5);
                }
                if (gamepad1.x) {
                    linearGripper.setPosition(.01);
                }

                //Arm
                if(armLocked == true){
                    armRotate.setPower(0.05);
                    armBrace.setPower(0.05);
                } else{
                    if (gamepad1.right_bumper) {
                        armRotate.setPower(0.3);
                        armBrace.setPower(0.3);
                    } else if (gamepad1.left_bumper) {
                        armRotate.setPower(-0.2);
                        armBrace.setPower(-0.2);
                    } else {
                        armRotate.setPower(0);
                        armBrace.setPower(0);
                    }
                }

                if (ext > 350 && ext < 2900) {
                    if (gamepad1.dpad_up) {
                        armExt.setPower(1);
                    } else if (gamepad1.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext < 350 && ext < 2900) {
                    if (gamepad1.dpad_up) {
                        armExt.setPower(1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext > 2900 && ext > 350) {
                    if (gamepad1.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } if(gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5){
                    armExt.setPower(-1);
                    sleep(1000000);
                }
            }
        }
    }

}

