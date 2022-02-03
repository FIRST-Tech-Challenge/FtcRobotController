package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

public class DistanceTimeoutException extends MovementException {

    private final ElapsedTime timer;
    private final double tooSmallOfDistance; // this travels ~2 inches for every 1000 millis
    private final double millis;
    private double positionAfterTimeLoop = Double.MAX_VALUE; //These are arrays to make the compiler happy. Treat them as a normal double


    public DistanceTimeoutException(double millis) {
        super();
        timer = new ElapsedTime();
        tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis
        this.millis = millis;

    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (timer.milliseconds() >= millis) {
            //These are arrays to make the compiler happy. Treat them as a normal double
            double positionBeforeTimeLoop = positionAfterTimeLoop;
            positionAfterTimeLoop = MiscUtils.distance(gps.getX(), gps.getY(), x, y);
            double traveledDistance = Math.abs(positionBeforeTimeLoop - positionAfterTimeLoop);
            if (traveledDistance < tooSmallOfDistance) {
                final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
                final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Exception.\n";
                RobotLog.addGlobalWarningMessage(errorMsg);
                throw this;
            }
            timer.reset();
        }
    }
}
