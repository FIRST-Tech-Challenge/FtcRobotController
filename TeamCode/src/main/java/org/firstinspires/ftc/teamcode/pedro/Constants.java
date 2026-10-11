package org.firstinspires.ftc.teamcode.pedro;

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
        c.frontLeftName.set("leftFrontMotor");
        c.frontRightName.set("rightFrontMotor");
        c.backLeftName.set("leftBackMotor");
        c.backRightName.set("rightBackMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    ;
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.6085158821166035);
        c.yPodOffset.set(2.231271999088798);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.14328138767435716);
                Controller secondaryTranslationalForward = Controller.proportional(0.052938644196484905);
                Controller primaryTranslationalLateral = Controller.proportional(0.18212077301289095);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06728875927158333);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016925312435587862));
                c.brake.set(Controller.proportionalFeedforward(0.014386515570249683));

                c.headingFeedback.set(Controller.proportional(2.546524106351035));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04303836669843841, 0.004573106303051341));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06295202740943867, 0.04193621161195966));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010301143778874975, 0.0014340913065435143));

                c.maxAchievableForwardVelocity.set(62.40996894074);
                c.maxAchievableStrafeVelocity.set(53.77263468700155);
                c.naturalForwardDeceleration.set(24.611048594153946);
                c.naturalStrafeDeceleration.set(36.28069148574015);
            }
    );
}