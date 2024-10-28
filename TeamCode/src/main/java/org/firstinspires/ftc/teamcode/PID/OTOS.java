package org.firstinspires.ftc.teamcode.PID;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class OTOS {

    private final SparkFunOTOS otos;

    public OTOS(SparkFunOTOS otosSensor) {
        otos = otosSensor;
    }

    @SuppressLint("DefaultLocale")
    public void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);  // configure
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

}
