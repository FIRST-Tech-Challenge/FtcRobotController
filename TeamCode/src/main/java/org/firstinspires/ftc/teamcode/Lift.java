package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    static int target = 0;
    static void lift(Project1Hardware robot, Gamepad gamepad1, Telemetry telemetry){
        if (gamepad1.dpad_up) {
            target += 50;
        }
        if (gamepad1.dpad_down) {
            target += -50;
        }
        robot.vert.setTargetPosition(target);
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
