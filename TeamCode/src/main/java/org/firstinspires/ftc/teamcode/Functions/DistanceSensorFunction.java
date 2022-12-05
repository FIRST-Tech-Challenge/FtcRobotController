package org.firstinspires.ftc.teamcode.Functions;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorFunction {

    private DistanceSensor sensorRange;

    public DistanceSensorFunction(DistanceSensor _sensorRange)
    {
        sensorRange = _sensorRange;
    }


    //get ma of 10 measures of distance
    public double distance()
    {
        double value2 = 0;

        for (int i = 0; i < 10; i++) {
            double value = sensorRange.getDistance(DistanceUnit.CM);

            if (value < 819)
                value2 += value;
            else value2 += 0;
        }

        value2 /= 10;

        return value2;
    }

}


//pt folosire int alte clase, sa se adauge urmatoarele linii:

//    @Override
//    public void init() {
//        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
//        distanceSensorFunction = new DistanceSensorFunction(sensorRange);
//
//    }

// + apel functie "distanceSensorFunction.distance();"

//senzorul nu compenseaza pt marimea lui (+- 2 cm)
