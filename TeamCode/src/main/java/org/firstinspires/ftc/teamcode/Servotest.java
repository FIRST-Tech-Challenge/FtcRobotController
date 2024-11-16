package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware.MID_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servotest")
public class Servotest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.tryGet(Servo.class, "wrist");
        waitForStart();
        while (opModeIsActive()) { //while loop for when program is active
            if(gamepad1.y) {
                servo.setPosition(MID_SERVO);

            }
            if(gamepad1.x){
                servo.setPosition(.4725);

            }
            if(gamepad1.b) {
                servo.setPosition(.855);
            }
        }
    }
}
