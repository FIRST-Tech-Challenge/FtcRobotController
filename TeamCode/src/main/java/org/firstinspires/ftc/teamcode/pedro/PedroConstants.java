package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

// Pedro's AutoTune prints Foresight settings that use these three. They are imported now so
// the pasted lines compile without hunting for imports.
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * Pedro Pathing's settings for this robot, and the one place that builds a {@link Follower}.
 *
 * <p>Pedro needs three things (docs/decisions/0008):
 * <ol>
 *   <li><b>Drivetrain</b>: which motors, and which way they spin. {@link #DriveConfig}
 *   <li><b>Localizer</b>: how the robot knows where it is. A goBILDA Pinpoint when the robot
 *       has one ({@link #PinpointLocalizerConfig}), else two odometry pods on the hub plus the
 *       Control Hub IMU ({@link #TwoWheelLocalizerConfig}). {@link #CreateLocalizer} picks.
 *   <li><b>Foresight</b>: how hard to push to stay on the path and stop on time. These numbers
 *       only come from running AutoTune on the real robot. {@link #ForesightConfig()}
 * </ol>
 *
 * <p>Tuning order (AutoTune: join the robot's Wi-Fi, open http://192.168.43.1:10158):
 * Mecanum Tuner, then Pinpoint Tuner (or Two Wheel Tuner without a Pinpoint), then Foresight
 * Tuner. After each one, copy the lines
 * from AutoTune's Java tab into the matching method below. They all start with {@code c.},
 * which is why the settings variable below is named {@code c}.
 *
 * <p>The motor names, pod ports, and hub mounting are the robot's, not Pedro's, so they come
 * from {@link BotProperties}. Only Pedro's own numbers live in this file.
 */
public final class PedroConstants {
    private PedroConstants() {
    }

    /**
     * false until the Foresight Tuner has been run and its numbers pasted into
     * {@link #ForesightConfig()}. The shape autos refuse to move while this is false.
     */
    public static final boolean foresightTuned = true;

    /** The four drive motors. Names and directions match {@code subsystems/Drive.java}. */
    public static MecanumConfig DriveConfig(BotProperties properties) {
        // Pedro wants its settings handed over inside a "settings -> { }" block. We give it an
        // empty one, then fill the settings in one line at a time.
        MecanumConfig c = new MecanumConfig(settings -> { });
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        return c;
    }

    /**
     * Two odometry pods plugged into hub encoder jacks, plus the Control Hub IMU for heading.
     *
     * <p>Until the Two Wheel Tuner has been run, these are starting guesses. The inches per
     * tick is Decode's measured value for these same pods. Offsets 0 and FORWARD directions
     * are placeholders.
     */
    public static TwoWheelConfig TwoWheelLocalizerConfig(BotProperties properties) {
        TwoWheelConfig c = new TwoWheelConfig(settings -> { });
        c.xPodName.set(properties.cfgForwardPodPort);
        c.yPodName.set(properties.cfgStrafePodPort);
        c.imuName.set(properties.cfgHubImu);
        c.forwardTicksToInches.set(properties.pinpointInPerTick);
        c.strafeTicksToInches.set(properties.pinpointInPerTick);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        RevHubOrientationOnRobot hubMounting =
                new RevHubOrientationOnRobot(properties.hubLogoFacing, properties.hubUsbFacing);
        c.imu.set(new RevHubIMU(hubMounting));
        return c;
    }

    /**
     * The goBILDA Pinpoint. Both pods plug into it, and it works out position and heading
     * by itself. Where the pods sit and how many ticks make an inch are the same numbers
     * {@code Imu_Pinpoint} uses, so there is one place to change them: BotProperties.
     *
     * <p>Until the Pinpoint Tuner has been run, the pod directions are a guess. Paste its
     * lines here.
     */
    public static PinpointConfig PinpointLocalizerConfig(BotProperties properties) {
        PinpointConfig c = new PinpointConfig(settings -> { });
        c.name.set(properties.cfgPinpoint);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Inches from the robot's center to each pod: the forward pod sideways (left is
        // positive), the strafe pod forwards.
//        c.xPodOffset.set(properties.pinpointParYTicks * properties.pinpointInPerTick);
//        c.yPodOffset.set(properties.pinpointPerpXTicks * properties.pinpointInPerTick);
//        c.xPodOffset.set(3.11840417816883);
//        c.yPodOffset.set(-3.502030860720657);
        c.xPodOffset.set(4.9890695406695995);
        c.yPodOffset.set(1.2506654694324404);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        return c;
    }

    /** true when a device named {@code cfgPinpoint} is in the robot configuration. */
    public static boolean UsesPinpoint(HardwareMap hardwareMap, BotProperties properties) {
        return hardwareMap.tryGet(GoBildaPinpointDriver.class, properties.cfgPinpoint) != null;
    }

    /**
     * How the robot knows where it is: the Pinpoint when there is one, else two pods on the
     * hub. The same swap {@code ImuSelector} makes for the heading in TeleOp.
     */
    public static Localizer CreateLocalizer(HardwareMap hardwareMap, BotProperties properties) {
        if (UsesPinpoint(hardwareMap, properties)) {
            return new PinpointLocalizer(hardwareMap, PinpointLocalizerConfig(properties));
        }
        return new TwoWheelLocalizer(hardwareMap, TwoWheelLocalizerConfig(properties));
    }

    /**
     * How Pedro follows a path. Empty until the Foresight Tuner has been run: paste the
     * {@code c.} lines from AutoTune's Java tab here, then set {@link #foresightTuned} to true.
     */
    public static ForesightConfig ForesightConfig() {
        ForesightConfig c = new ForesightConfig(settings -> { });
        // PASTE the Foresight Tuner's lines here.
        Controller primaryTranslationalForward = Controller.proportional(0.15771649799694493);
        Controller secondaryTranslationalForward = Controller.proportional(0.05827203174742949);
        Controller primaryTranslationalLateral = Controller.proportional(0.2023418720879037);
        Controller secondaryTranslationalLateral = Controller.proportional(0.07475991506208207);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01814835306805315));
        c.brake.set(Controller.proportionalFeedforward(0.015426100107845175));

        c.headingFeedback.set(Controller.proportional(3.2558727073334492));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04409037864527982, 0.0031776398039946763));

        c.linearBrakeCoefficients.set(Matrix.diag(0.0551406135450986, 0.05252823212691348));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011862051705027507, 0.001136887238096803));

        c.maxAchievableForwardVelocity.set(57.19474425053879);
        c.maxAchievableStrafeVelocity.set(50.64378307473817);
        c.naturalForwardDeceleration.set(34.74800720443923);
        c.naturalStrafeDeceleration.set(50.822903355904316);
        return c;
    }

    /**
     * Build a Follower: the part of Pedro that drives the robot along a path. OpModes get
     * theirs from {@code Bot.CreateFollower()}, which fills these in.
     *
     * @param properties the robot's settings: motor names, pod ports, hub mounting
     * @param safeMode true: every drive power is limited to the Safe Mode speed (decision 0004)
     */
    public static Follower CreateFollower(HardwareMap hardwareMap, BotProperties properties,
                                          boolean safeMode) {
        Localizer localizer = CreateLocalizer(hardwareMap, properties);
        Mecanum drivetrain = new SafeModeMecanum(hardwareMap, DriveConfig(properties), safeMode);
        Foresight algorithm = new Foresight(ForesightConfig());
        return new Follower(localizer, drivetrain, algorithm);
    }
}
