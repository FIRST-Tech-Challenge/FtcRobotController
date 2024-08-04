package org.firstinspires.ftc.teamcode.mmooover;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderTracking {

    private double tick2inch(int ticks) {
        return (ticks / ticksPerRotation) * 2 * PI * radiusInches;
    }

    private final double ticksPerRotation;
    private final double radiusInches;
    private final double trackWidth; // distance between two parallel encoders
    private final double forwardOffset;

    double heading = 0;
    double x = 0;
    double y = 0;

    double lastLeft, lastCenter, lastRight;
    final DcMotor leftEncoder, centerEncoder, rightEncoder;

    public EncoderTracking(TriOdoProvider encoderSource) {
        leftEncoder = encoderSource.getLeftEncoder();
        centerEncoder = encoderSource.getCenterEncoder();
        rightEncoder = encoderSource.getRightEncoder();
        ticksPerRotation = encoderSource.getEncoderTicksPerRevolution();
        radiusInches = encoderSource.getEncoderWheelRadius();
        trackWidth = encoderSource.getTrackWidth();
        forwardOffset = encoderSource.getForwardOffset();

        lastLeft = tick2inch(leftEncoder.getCurrentPosition());
        lastCenter = tick2inch(centerEncoder.getCurrentPosition());
        lastRight = tick2inch(rightEncoder.getCurrentPosition());
    }

    public void step() {
        double currentLeft = tick2inch(leftEncoder.getCurrentPosition());
        double currentCenter = tick2inch(centerEncoder.getCurrentPosition());
        double currentRight = tick2inch(rightEncoder.getCurrentPosition());

        double deltaLeft = currentLeft - lastLeft;
        double deltaCenter = currentCenter - lastCenter;
        double deltaRight = currentRight - lastRight;

        double deltaTurn = (deltaRight - deltaLeft) / trackWidth;
        double deltaForward = (deltaLeft + deltaRight) / 2.;
        double deltaStrafe = deltaCenter - forwardOffset * deltaTurn;

        double nextHeading = heading + deltaTurn;
        double avgHead = (heading + nextHeading) / 2.;
        double deltaX = deltaForward * cos(avgHead) - deltaStrafe * sin(avgHead);
        double deltaY = deltaForward * sin(avgHead) + deltaStrafe * cos(avgHead);

        x += deltaX;
        y += deltaY;
        heading = nextHeading;

        lastLeft = currentLeft;
        lastCenter = currentCenter;
        lastRight = currentRight;
    }

    public Pose getPose() {
        return new Pose(
                x, y, heading
        );
    }
}
