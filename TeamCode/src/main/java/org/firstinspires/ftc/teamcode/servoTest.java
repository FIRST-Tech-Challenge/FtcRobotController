package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class servoTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("servo1");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            while (gamepad1.a) {
                servo.setPosition(0);
                wait(1);
                servo.setPosition(1);
                wait(1);
            }
        }
    }
}
