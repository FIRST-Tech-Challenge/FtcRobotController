package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

import androidx.annotation.NonNull;

public class AutoLift extends ControllerLift {
    // fix this later
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     * @param toolGamepad instance of FtcLib GamepadEx
     * @param telemetry   local telemetry instance
     */
    public AutoLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, Telemetry telemetry) {
        super(eventThread, map, toolGamepad, telemetry);
    }
    // TODO make lift aaaaAAAAAAAAAA
}
