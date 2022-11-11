package org.firstinspires.ftc.teamcode.Functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "DistanceSeonsorTeleOp", group = "TEST")
public class DIstanceSensor extends OpMode {

    private DistanceSensor sensorRange;
    private DistanceSensorFunction distanceSensorFunction;


    @Override
    public void init() {
//        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        distanceSensorFunction = new DistanceSensorFunction(sensorRange);

    }

    @Override
    public void loop() {

        double valoare = distanceSensorFunction.distance();
        telemetry.addData("Distanta: ", valoare);

    }
}
