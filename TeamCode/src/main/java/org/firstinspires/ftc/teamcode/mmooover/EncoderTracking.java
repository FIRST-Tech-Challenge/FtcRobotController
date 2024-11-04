package org.firstinspires.ftc.teamcode.mmooover;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Encoder;

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
    final Encoder leftEncoder, centerEncoder, rightEncoder;

    // TriOdoProvider provides the methods for the getting the encoders
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
    // Updates the pose
    public void step() {
        // Current encoder values
        double currentLeft = tick2inch(leftEncoder.getCurrentPosition());
        double currentCenter = tick2inch(centerEncoder.getCurrentPosition());
        double currentRight = tick2inch(rightEncoder.getCurrentPosition());
        // Change in the encoders' values
        double deltaLeft = currentLeft - lastLeft;
        double deltaCenter = currentCenter - lastCenter;
        double deltaRight = currentRight - lastRight;
        // Change in distances
        double deltaTurn = (deltaRight - deltaLeft) / trackWidth;
        double deltaForward = (deltaLeft + deltaRight) / 2.;
        double deltaStrafe = deltaCenter - forwardOffset * deltaTurn;
        // Predicts how much our robot turns so we know how much forward and strafe we now need - the heading changes enough in each loop that we need to predict it
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
// Gets current pose
    public Pose getPose() {
        return new Pose(
                x, y, heading
        );
    }
}
