package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConeDetectorSubsystem extends SubsystemBase {
    private final SensorRevTOFDistance distSensor;
    private final double target;

    private boolean isEnabled = true;

    public ConeDetectorSubsystem(HardwareMap hm, double target) {
        DistanceSensor sensor = (DistanceSensor) hm.get(NormalizedColorSensor.class,
                "cone_detector");
        distSensor = new SensorRevTOFDistance(sensor);

        this.target = target;
    }

    public double getDistance() {
        return distSensor.getDistance(DistanceUnit.MM);
    }

    public void toogleState() {
        isEnabled = !isEnabled;
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public Boolean isConeDetected() {
        return getDistance() <= target;
    }
}