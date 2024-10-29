package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SuperOp")
public class superOp extends OpMode {
    public SparkFunOTOS odo;
    @Override
    public void init() {
        odo = hardwareMap.get(SparkFunOTOS.class, "odometry");
    }

    @Override
    public void loop() {
        telemetry.addData("x", odo.getPosition().x);
        telemetry.addData("y", odo.getPosition().y);
        telemetry.addData("h", odo.getPosition().h);
    }
}
