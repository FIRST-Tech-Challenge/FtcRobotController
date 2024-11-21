package org.firstinspires.ftc.teamcode.Debug;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Debug {
    public boolean debugMode = false;
    private final OpMode opMode;
    private final Telemetry telemetry;

    public Debug(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    public void checkDebugButtons(Gamepad gamepad) {
        if (gamepad.start && gamepad.back) {
            debugMode = !debugMode;
            telemetry.addData("debug mode: ", debugMode);
            telemetry.update();
        }
    }

    public boolean getDebugMode() {
        return debugMode;
    }

    public void outputDebugInfo(String message) {
        if (debugMode) {
            telemetry.addData("Debug Info: ", message);
        }
        else {
            telemetry.addData("Debug Info: ", "Debug mode is off");
        }
        telemetry.update();
    }

}
