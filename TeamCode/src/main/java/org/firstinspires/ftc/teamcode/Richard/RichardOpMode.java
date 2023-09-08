package org.firstinspires.ftc.teamcode.Richard;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class RichardOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Servo Left = hardwareMap.servo.get("Left");
        Servo Right = hardwareMap.servo.get("Right");

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Left.setPosition(0.5);
                Right.setPosition(0);
            }
            if (gamepad1.b) {
                Left.setPosition(0);
                Right.setPosition(0.5);
            }
        }
    }
}
