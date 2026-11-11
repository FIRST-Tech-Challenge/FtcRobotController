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
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.760870445431687); //original -4.71904334120863
        c.yPodOffset.set(-1.0819689683088167); //original -0.8108188599113404
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2754472410746232);
                Controller secondaryTranslationalForward = Controller.proportional(0.10177039549123913);
                Controller primaryTranslationalLateral = Controller.proportional(0.45665079823067695);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16872026801218015);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011920294624860078));
                c.brake.set(Controller.proportionalFeedforward(0.010132250431131066));

                c.headingFeedback.set(Controller.proportional(4.02893546007933));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05599127981347979, 0.003745240565467153));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12748849523729622, 0.060276409386388105));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-2.0434554928685998E-4, 0.0015485997658823897));

                c.maxAchievableForwardVelocity.set(82.06646195889867);
                c.maxAchievableStrafeVelocity.set(68.48802187322583);
                c.naturalForwardDeceleration.set(28.073897000954467);
                c.naturalStrafeDeceleration.set(57.711316592353825);

                c.brakeAggression.set(1.2); // makes the autos overshoot a little
            }
    );


}