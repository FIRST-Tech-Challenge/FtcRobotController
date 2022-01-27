package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationErrors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

public class TimeoutError implements NavigationError {

    final ElapsedTime t;
    final double timeout;

    public TimeoutError(double timeout) {
        t = new ElapsedTime();
        t.reset();
        this.timeout = timeout;
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (t.seconds() > timeout) {
            final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
            final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Error.\n";
            throw new MovementException(errorMsg);
        }
    }
}
