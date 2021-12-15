package org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.eyesquaredsea.EyeSquaredSeaDistanceSensor;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.SensorWrapper;

public class UltrasonicDistanceSensor implements SensorWrapper {
    public EyeSquaredSeaDistanceSensor sensor;
    public DistanceUnit units = DistanceUnit.CM;
    public String name;
    public LinearOpMode opMode;

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String name, LinearOpMode opMode) {
        this.name = name;
        sensor = hardwareMap.get(EyeSquaredSeaDistanceSensor.class, name);
        this.opMode = opMode;
    }

    @Override
    public void setUnits(DistanceUnit unit) {
        units = unit;
    }

    @Override
    public int getData() {
        sensor.writeData(EyeSquaredSeaDistanceSensor.Commands.WRITE_RANGE_COMMAND);
        opMode.sleep(100);
        return (int) sensor.readData(EyeSquaredSeaDistanceSensor.Commands.READ_LAST.bVal);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean didTimeoutOccur() {
        return false;
    }
}
