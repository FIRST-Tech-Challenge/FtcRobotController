package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Odometry {

    private final SparkFunOTOS otos;

    public Odometry(SparkFunOTOS otosSensor) {
        otos = otosSensor;

        // (incomplete) config; offset depends on where we mount, scalars need tuning
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();

    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }
}
