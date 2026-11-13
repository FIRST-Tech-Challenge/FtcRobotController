package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FrontLM");
        c.frontRightName.set("FrontRM");
        c.backLeftName.set("BackLM");
        c.backRightName.set("BackRM");
        c.frontLeftDirection.set(DcMotor.Direction.REVERSE);
        c.frontRightDirection.set(DcMotor.Direction.FORWARD);
        c.backLeftDirection.set(DcMotor.Direction.REVERSE);
        c.backRightDirection.set(DcMotor.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.7913483597162202);
        c.yPodOffset.set(1.373716489536556);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17430634500386044);
                Controller secondaryTranslationalForward = Controller.proportional(0.06440153692760858);
                Controller primaryTranslationalLateral = Controller.proportional(0.22594035072789526);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08347892235659646);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017554285353381896));
                c.brake.set(Controller.proportionalFeedforward(0.01492114255037461));

                c.headingFeedback.set(Controller.proportional(1.9705183816629792));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03436075144320541, 0.014115877754955347));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06892939296504882, 0.04016964321740206));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010896604839309625, 0.001889952924503646));

                c.maxAchievableForwardVelocity.set(37.588243083446926);
                c.maxAchievableStrafeVelocity.set(33.24281676927984);
                c.naturalForwardDeceleration.set(56.42420833324964);
                c.naturalStrafeDeceleration.set(65.97906503805277);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}