package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "DistanceSeonsorTeleOp", group = "TEST")
public class DIstanceSensor extends OpMode {

    private DistanceSensor sensorRange;
    private DistanceSensorFunction distanceSensorFunction;

    @Override
    public void init() {
        distanceSensorFunction = new DistanceSensorFunction(sensorRange);
    }

    @Override
    public void loop() {

        double valoare = distanceSensorFunction.distance();
        telemetry.addData("Distanta: ", valoare);

    }
}
