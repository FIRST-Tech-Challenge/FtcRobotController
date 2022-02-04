package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

/**
 * If the total time elapsed for the movement is greater than timeout, it throws
 */
public class TimeoutException extends MovementException {

    final ElapsedTime t;
    final double timeout;


    public TimeoutException(double millis) {
        super();
        t = new ElapsedTime();
        t.reset();
        this.timeout = millis;
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (t.milliseconds() > timeout) {
            final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
            final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Exception.\n";
            RobotLog.addGlobalWarningMessage(errorMsg);
            throw this;
        }
    }
}
