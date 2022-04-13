package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

//@Disabled
@Config
@TeleOp(name = "\uFFFFEli Outtake Test😂")
public class EliOuttakeTest extends GenericOpModeTemplate {

    public static double closedPos = 1;

    public static double openPos = 0.9;

    @Override
    public void opModeMain() {
        Servo outtakeServo = hardwareMap.servo.get(GenericOpModeTemplate.bucketServoName);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                outtakeServo.setPosition(closedPos);
            } else if (gamepad2.b) {
                outtakeServo.setPosition(openPos);
            }
        }

    }
}
