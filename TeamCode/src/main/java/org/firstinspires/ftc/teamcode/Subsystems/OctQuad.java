package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class OctQuad extends SubsystemBase {

    // Odometry pods - constants
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.024; // m
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 0.315; // m; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0.100; // m; offset of the lateral wheel

    // travel distnce per encoder pulse (m)
    private double DistancePerPulse = Math.PI * 2.0 * WHEEL_RADIUS/(TICKS_PER_REV*GEAR_RATIO);

    // Create octoquad object
    OctoQuad octoquad;

    // encoder values from each odometry pod
    double OdometryLeftEncoder;
    double OdometryRightEncoder;
    double OdometryFrontEncoder;

    /** Place code here to initialize subsystem */
    public OctQuad() {
        // set up octoquad object
        octoquad = RobotContainer.ActiveOpMode.hardwareMap.get(OctoQuad.class, "OctoQuad");

        // reset octoquad
        octoquad.resetEverything();

        // reset all encoders
        octoquad.resetAllPositions();

        // set encoder directions
        octoquad.setSingleEncoderDirection(0, OctoQuadBase.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(1, OctoQuadBase.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(2, OctoQuadBase.EncoderDirection.FORWARD);

        // set all encoders to quadrature
        octoquad.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_QUADRATURE);

        // save update to flash
        octoquad.saveParametersToFlash();

        // reset all encoders
        OdometryLeftEncoder=0;
        OdometryRightEncoder=0;
        OdometryFrontEncoder=0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // read encoder values
//        TODO
//        OdometryRightEncoder = octoquad.readSinglePosition(0);
//        OdometryLeftEncoder = octoquad.readSinglePosition(1);
//        OdometryFrontEncoder = octoquad.readSinglePosition(2);
    }

    // methods to return encoder values of odometry pods
    public double getLeftEncoderDistance() {
        return DistancePerPulse*OdometryLeftEncoder;
    }
    public double getRightEncoderDistance() {
        return DistancePerPulse*OdometryRightEncoder;
    }
    public double getFrontEncoderDistance() {
        return DistancePerPulse*OdometryFrontEncoder;
    }

}