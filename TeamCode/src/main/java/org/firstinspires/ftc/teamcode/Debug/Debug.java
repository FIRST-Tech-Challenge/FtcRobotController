package org.firstinspires.ftc.teamcode.Debug;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Debug {
    public boolean debugMode = false;
    OpMode opMode;

    public Debug(OpMode opmode) {
        this.opMode = opmode;
    }

    public void checkDebugButtons(Gamepad gamepad) {
        if (gamepad.start && gamepad.back) {
            debugMode = !debugMode;
            opMode.telemetry.addData("debug mode: ", debugMode);
            opMode.telemetry.update();
        }
    }

    public boolean getDebugMode() {
        return debugMode;
    }

    public void outputDebugInfo(String message) {
        if (debugMode) {
            opMode.telemetry.addData("Debug Info: ", message);
            opMode.telemetry.update();
        }
        else {
            opMode.telemetry.addData("Debug Info: ", "Debug mode is off");
            opMode.telemetry.update();
        }
    }

}
