package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.JNIGetString;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;


@TeleOp(name = "JNITest \uD83D\uDE80")
public class JNITest extends GenericOpModeTemplate {
    @Override
    public void opModeMain() {
        waitForStart();

        System.loadLibrary("JNIGetStr");

        while (opModeIsActive()) {
            telemetry.addData("JNI Str", JNIGetString.stringFromJNI());
            telemetry.addData("Extra Call Res", JNIGetString.extraCallFromJNI(JNIGetString.getInstance()));
            telemetry.addData("Num", JNIGetString.intFromJNI());
            telemetry.update();
        }
    }
}
