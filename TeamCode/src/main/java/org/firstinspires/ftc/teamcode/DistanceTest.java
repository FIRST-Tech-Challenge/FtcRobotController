package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.TestDistance;

@TeleOp
public class DistanceTest extends OpMode {
TestDistance testDistance = new TestDistance();
    @Override
    public void init() {
        testDistance.init(hardwareMap);
    }

    @Override
    public void loop() {
       telemetry.addData("Distance", testDistance.getDistance());
    }
}
