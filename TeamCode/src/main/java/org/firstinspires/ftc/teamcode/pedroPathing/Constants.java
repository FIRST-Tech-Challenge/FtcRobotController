package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 configuration for Lox (the BioBuzzOpMode robot).
 *
 * Build a Follower with:
 *     Follower follower = Constants.create(hardwareMap);
 *
 * The drivetrain and localizer below are real values for this robot. THE FORESIGHT CONSTANTS
 * ARE NOT - they are the example numbers from Pedro's docs, measured on somebody else's robot,
 * and they are here only so that create() returns a usable Follower and so AutoTune has
 * something to paste over. Paths will not track accurately until they are replaced.
 *
 * To replace them, run AutoTune (see Tuning.java) and copy the generated block out of its
 * Java tab. Everything in this file is meant to be overwritten by AutoTune output.
 */
public class Constants {

    /*
     * Motor names and directions are copied from MecanumDrive.java so that Pedro drives Lox
     * the same way the TeleOp does. If a motor is ever re-wired or re-configured, change it
     * in BOTH places - they are independent.
     *
     * manualBrakeMode makes the drivetrain brake rather than coast when manual() is given zero
     * power, matching MecanumDrive's BRAKE zero-power behavior.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.frontRightName.set("frontRight");
                c.backLeftName.set("backLeft");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    /*
     * Taken from the values already tuned in SensorGoBildaPinpoint:
     *     setOffsets(154.0, -81.0, DistanceUnit.MM)
     *     setEncoderResolution(goBILDA_4_BAR_POD)
     *     setEncoderDirections(FORWARD, FORWARD)
     *
     * PinpointLocalizer passes xPodOffset and yPodOffset straight through to the driver's
     * setOffsets(x, y, unit), so these map one-to-one. offsetUnits applies only to the two
     * offsets, which is why they can stay in millimetres here; globalDistanceUnit (inches)
     * is the separate unit every pose is reported in.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(156.0);
                c.yPodOffset.set(-81.0);
                c.offsetUnits.set(DistanceUnit.MM);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    /*
     * PLACEHOLDER - not Lox's numbers. See the class comment. Pedro 3 gives these no defaults
     * (they are ConfigVar.required()), so something has to be here for a Follower to exist at
     * all; these are the sample values from https://pedropathing.com/docs/pathing/tuning.
     *
     * Run the Foresight tuner in AutoTune and paste its output over this whole block.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

                c.headingFeedback.set(Controller.proportional(5.258721785960744));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
