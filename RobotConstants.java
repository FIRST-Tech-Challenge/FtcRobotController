package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;


public class RobotConstants {

    // Define standard fallback configurations directly to get tracking live immediately
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");   // Match these exactly with Driver Station config configuration names
        c.backLeftName.set("leftRear");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightRear");
    });

    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.brakeAggression.set(1.0);
        c.maxBrakingPower.set(0.2);
    });

    /**
     * Instantiates the core Pedro Pathing system using your 2-wheel dead wheel configuration.
     */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, new com.pedropathing.revhub.localizers.TwoWheelConfig(c -> {})),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
