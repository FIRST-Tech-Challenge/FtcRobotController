package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class SampleDetector {

    private int bufferCounter = 0;
    private double[] buffer = new double[5];

    private RevColorSensorV3 colorSensor;

    public SampleDetector (Hardware hardware) {
        colorSensor = hardware.intakeCS;
    }

    public boolean sampleDetected() {
        return (aO5() <= IntakeConstants.detectionDistance);
    }

    public double getDistance() {
        return aO5();
    }

    public void update() {
        buffer[bufferCounter % 4] = colorSensor.getDistance(DistanceUnit.MM);
        bufferCounter += 1;
    }

    private double aO5() {
        double total = 0.00;

        for (double distance : buffer) {
            if (distance != 0) {
                total += distance;
            }
        }

        return total / 5;
    }
}
