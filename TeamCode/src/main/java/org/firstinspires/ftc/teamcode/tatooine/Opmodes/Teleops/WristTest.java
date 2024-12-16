package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WristTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "crServo");
        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(1);
        }
    }
}
