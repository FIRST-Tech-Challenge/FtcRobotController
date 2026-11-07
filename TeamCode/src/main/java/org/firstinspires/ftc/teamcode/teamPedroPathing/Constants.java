package org.firstinspires.ftc.teamcode.teamPedroPathing;

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

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LeftFront");
        c.frontRightName.set("FrontRight");
        c.backLeftName.set("LeftBack");
        c.backRightName.set("BackRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD );

            c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.8370592312549983);
        c.yPodOffset.set(-0.06656290039302797);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2196602649043238);
                Controller secondaryTranslationalForward = Controller.proportional(0.08115859845177056);
                Controller primaryTranslationalLateral = Controller.proportional(0.4411715449809015);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16300109754965258);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01396200054024545));
                c.brake.set(Controller.proportionalFeedforward(0.011867700459208631));

                c.headingFeedback.set(Controller.proportional(4.082398796052427));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03405654792934925, 0.012302163400546167));

                c.linearBrakeCoefficients.set(Matrix.diag(0.020410859749015516, 0.058309515524160166));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.003797404033946162, 0.0021314935627607273));

                c.maxAchievableForwardVelocity.set(69.83306912224118);
                c.maxAchievableStrafeVelocity.set(55.42220540574516);
                c.naturalForwardDeceleration.set(28.486517909750596);
                c.naturalStrafeDeceleration.set(62.8942978440022);
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