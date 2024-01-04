package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="FossTeleop120623")



public class FossTeleop120623 extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_9;
    public DcMotorEx back_Slide;
    public DcMotorEx front_Slide;
    float front_LeftPower, back_LeftPower, front_RightPower, back_RightPower, drive, strafe, spin;
    public Servo airplane;
    public CRServo intake1, intake2;


    @Override
    public void runOpMode() {
        back_Left = hardwareMap.get(DcMotor.class, "back_Left");
        back_Right = hardwareMap.get(DcMotor.class, "back_Right");
        front_Left = hardwareMap.get(DcMotor.class, "front_Left");
        front_Right = hardwareMap.get(DcMotor.class, "front_Right");
        back_Slide = hardwareMap.get(DcMotorEx.class, "back_Slide");
        front_Slide = hardwareMap.get(DcMotorEx.class, "front_Slide");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_9 = hardwareMap.get(Blinker.class, "Expansion Hub 9");
        airplane = hardwareMap.get(Servo.class, "airplane");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        front_Right.setDirection(DcMotor.Direction.REVERSE);
        back_Right.setDirection(DcMotor.Direction.REVERSE);

        //  front_Slide.setDirection(DcMotorEx.Direction.REVERSE);
        airplane.setDirection(Servo.Direction.REVERSE);
        intake2.setDirection(CRServo.Direction.REVERSE);

        back_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            //arcade mode
            drive = gamepad1.right_stick_y;
            spin = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            front_LeftPower = drive + strafe - spin;
            back_LeftPower = drive - strafe - spin;
            front_RightPower = drive - strafe + spin;
            back_RightPower = drive + strafe + spin;

            front_Left.setPower(front_LeftPower * 0.6);
            back_Left.setPower(back_LeftPower * 0.6);
            front_Right.setPower(front_RightPower * 0.6);
            back_Right.setPower(back_RightPower * 0.6);

            if(gamepad1.dpad_up){
                front_Slide.setTargetPosition(1100);
                back_Slide.setTargetPosition(1100);
                front_Slide.setVelocity(1000);
                back_Slide.setVelocity(1000);
                front_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                //front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                telemetry.addData("front slide enc",front_Slide.getCurrentPosition());
                telemetry.update();

            }
            //    back_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //   front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            if(gamepad1.dpad_down){
                front_Slide.setTargetPosition(200);
                back_Slide.setTargetPosition(200);
                front_Slide.setVelocity(1000);
                back_Slide.setVelocity(1000);
                front_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                //front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("front slide enc",front_Slide.getCurrentPosition());
                telemetry.update();

                //   back_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //   front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("front_slide",front_Slide.getCurrentPosition());
            telemetry.addData("back_slide",back_Slide.getCurrentPosition());
            telemetry.update();

            if(gamepad2.x){
                airplane.setPosition(0);
            }

            while(gamepad2.left_trigger > .25){
                intake1.setPower(-1);
            }

            while(gamepad2.right_trigger > .25) {
                intake1.setPower(1);
            }

            intake1.setPower(0);


            /*
            if(gamepad1.dpad_right){
                back_Slide.setTargetPosition(500);
                back_Slide.setVelocity(1000);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //sleep(250);
                //front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                     telemetry.addData("back slide enc",back_Slide.getCurrentPosition());
                     telemetry.update();

            }

             if(gamepad1.dpad_left){
                //back_Slide.setTargetPosition(800);
                back_Slide.setTargetPosition(0);
                back_Slide.setVelocity(1000);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //sleep(250);
                //front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                //back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                     telemetry.addData("back slide enc",back_Slide.getCurrentPosition());
                     telemetry.update();
            */
        }
    }
}

