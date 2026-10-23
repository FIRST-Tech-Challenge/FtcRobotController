package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    /*  NOTICE --- The final line written under these static constants has to end in ";" and be the only one 'ended'.
    The order of testing is as follows; V V V
    === Start tuning with the TeleOp "Tuning" ! Scroll Down ! ===
    1. localization(PinpointConstants)   !!!DONE!!!
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants)   !!!DONE!!!
    === Start referring to Panels(192.168.43.1:8001) for values ===
    3. Translational PIDF (FollowerConstants)   !!! !!!
    4. Heading PIDF (FollowerConstants)   !!! !!!
    5. Drive PIDTF (FollowerConstants)   !!! !!!
    6. Centripetal Scale (FollowerConstants)   !!! !!!
    Done tuning! Go do tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)

    // TWEAK EVERYTHING FOR BIO-BUZZ!
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.5233344)
         // Tuning OpMode (Automatic Tuner)
            .forwardZeroPowerAcceleration(-28.568374693475533) // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            .lateralZeroPowerAcceleration(-79.64088451560347) // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
         // Dual PID System (after Tuning OpMode) if we want to use it, set following to "true"
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
         // Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.023, 0, 0.000495, 0.02675))
         // if using dual PIDF VVV (could be diff P,I,D,and F values)
         //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
         //  Heading (set NEW P, I, D, and F)
            .headingPIDFCoefficients(new PIDFCoefficients(0.585, 0, 0.002, 0.01))
         //if using dual PIDF VVV (could be diff P,I,D,and F values)
         //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
         //  Drive (set NEW P, I, D, T, and F)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0073, 0, 0.00003, 0.5, 0.004495))
         //if using dual PIDTF VVV (could be diff P, I, D, T, and F values)
         //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))
            .centripetalScaling(0.008)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Tuning Opmode (Automatic Tuners)
            .xVelocity(67.84071842704233) // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
           .yVelocity(55.14878448726624) // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Forward/Backward is +X/-X
            // Left/Right is +Y/-Y
            .forwardPodY(-(7.625)) // Distance from center (15.25/2) (y axis) // ||   // On back right
            .strafePodX((8.375)) // Distance from center (17.75/2) (x axis)  // --   // On back left
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ODO") // Ohh, Dee, Ohh (all capital)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // !test odo directions! if forward/left decrease, reverse them accordingly
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.74975, 1974.975, 1.001, 1.001);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}