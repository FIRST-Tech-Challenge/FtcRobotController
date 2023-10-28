package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevTouchSensor;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/**
 * William
 */
public class RFBreakBeam {
    private RevTouchSensor breakBeam;

    public RFBreakBeam(String p_breakBeamName) {
        breakBeam = op.hardwareMap.get(RevTouchSensor.class, p_breakBeamName);
    }

    /**
     * Checks if there is an object obstructing the beams.
     * Logs whether it found an object or not.
     * Logs to RFBreakBeam & general logs.
     * Logs to least fine level.
     * Does not update a state machine.
     */
    public boolean check() {
        LOGGER.log(RFLogger.Severity.INFO, "RFBreakBeam.check(): detected object: " + breakBeam.isPressed());
        return (breakBeam.isPressed());
    }
}
