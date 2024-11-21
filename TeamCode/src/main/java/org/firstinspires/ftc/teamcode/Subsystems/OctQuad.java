package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * The OctQuad class is a subsystem that manages the odometry pods using the OctoQuad hardware.
 * It extends the SubsystemBase class from FTCLib.
 */
public class OctQuad extends SubsystemBase {

    // Odometry pods - constants
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.024; // m
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 0.315; // m; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0.100; // m; offset of the lateral wheel

    // Travel distance per encoder pulse (m)
    private double DistancePerPulse = Math.PI * 2.0 * WHEEL_RADIUS / (TICKS_PER_REV * GEAR_RATIO);

    // Create octoquad object
    OctoQuad octoquad;

    // Encoder values from each odometry pod
    double OdometryLeftEncoder;
    double OdometryRightEncoder;
    double OdometryFrontEncoder;

    /**
     * Constructor for the OctQuad class.
     * Initializes the OctoQuad hardware and resets encoders.
     */
    public OctQuad() {
        // Set up octoquad object
        octoquad = RobotContainer.ActiveOpMode.hardwareMap.get(OctoQuad.class, "OctoQuad");

        // Reset octoquad
        octoquad.resetEverything();

        // Reset all encoders
        octoquad.resetAllPositions();

        // Set encoder directions
        octoquad.setSingleEncoderDirection(0, OctoQuadBase.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(1, OctoQuadBase.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(2, OctoQuadBase.EncoderDirection.FORWARD);

        // Set all encoders to quadrature
        octoquad.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_QUADRATURE);

        // Save update to flash
        octoquad.saveParametersToFlash();

        // Reset all encoders
        OdometryLeftEncoder = 0;
        OdometryRightEncoder = 0;
        OdometryFrontEncoder = 0;
    }

    /**
     * The periodic method is called periodically by the scheduler.
     * Reads the encoder values from the odometry pods.
     */
    @Override
    public void periodic() {
        // Read encoder values
        OdometryRightEncoder = octoquad.readSinglePosition(0);
        OdometryLeftEncoder = octoquad.readSinglePosition(1);
        OdometryFrontEncoder = octoquad.readSinglePosition(2);
    }

    /**
     * Returns the distance traveled by the left encoder.
     *
     * @return Distance traveled by the left encoder in meters.
     */
    public double getLeftEncoderDistance() {
        return DistancePerPulse * OdometryLeftEncoder;
    }

    /**
     * Returns the distance traveled by the right encoder.
     *
     * @return Distance traveled by the right encoder in meters.
     */
    public double getRightEncoderDistance() {
        return DistancePerPulse * OdometryRightEncoder;
    }

    /**
     * Returns the distance traveled by the front encoder.
     *
     * @return Distance traveled by the front encoder in meters.
     */
    public double getFrontEncoderDistance() {
        return DistancePerPulse * OdometryFrontEncoder;
    }
}
