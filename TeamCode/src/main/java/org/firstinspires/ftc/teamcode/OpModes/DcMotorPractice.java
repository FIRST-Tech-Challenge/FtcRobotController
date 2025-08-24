package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.TestBench1;

@TeleOp
public class DcMotorPractice extends OpMode {
    TestBench1 bench = new TestBench1();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.setMotorSpeed(0.5);
        telemetry.addData("Motor Revs", bench.getMotorRevs());
        
    }
}
