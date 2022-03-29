package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

@Disabled
@TeleOp(name = "\uFFFFEli Outtake Test😂")
public class EliOuttakeTest extends GenericOpModeTemplate {

    public static final double closedPos = 0;

    public static final double openPos = .5;

    @Override
    public void opModeMain() throws InterruptedException {
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
