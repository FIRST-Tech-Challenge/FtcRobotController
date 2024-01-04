package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arcade Centerstage")

public class ArcadeCenterstage extends LinearOpMode {
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

        back_Right.setDirection(DcMotor.Direction.REVERSE);
        front_Slide.setDirection(DcMotor.Direction.REVERSE);
        front_Right.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        back_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            //arcade mode
            drive = gamepad1.left_stick_y;
            spin = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            front_LeftPower = drive + strafe - spin;
            back_LeftPower = drive - strafe - spin;
            front_RightPower = drive - strafe + spin;
            back_RightPower = drive + strafe + spin;

            front_Left.setPower(front_LeftPower);
            back_Left.setPower(back_LeftPower);
            front_Right.setPower(front_RightPower);
            back_Right.setPower(back_RightPower);

            if (gamepad1.dpad_up) {
                front_Slide.setTargetPosition(1100);
                back_Slide.setTargetPosition(1100);
                front_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (front_Slide.isBusy()) {
                    front_Slide.setVelocity(1000);
                    back_Slide.setVelocity(1000);
                }
            }

            if (gamepad1.dpad_down) {
                front_Slide.setTargetPosition(200);
                back_Slide.setTargetPosition(200);
                front_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                back_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (front_Slide.isBusy()) {
                    front_Slide.setVelocity(1000);
                    back_Slide.setVelocity(1000);
                }
            }

            if (gamepad1.x) {
                airplane.setPosition(0.25);
            }
        }
    }
}