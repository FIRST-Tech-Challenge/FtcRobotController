package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestColorDistanceSensor extends TestItem {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public TestColorDistanceSensor(String description,
                                   ColorSensor colorSensor,
                                   DistanceSensor distanceSensor){
        super(description);
        this.colorSensor = colorSensor;
        this.distanceSensor = distanceSensor;
    }

    @Override
    public void run(boolean on, Telemetry telemetry){
        telemetry.addData("Amount Red", colorSensor.red());
        telemetry.addData("Amount Blue", colorSensor.blue());
        telemetry.addData("Distance (CM)", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance(IN)", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
