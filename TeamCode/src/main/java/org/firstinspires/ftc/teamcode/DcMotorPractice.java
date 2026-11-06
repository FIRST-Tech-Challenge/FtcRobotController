package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.TestBench;

@TeleOp
public class DcMotorPractice extends OpMode {

    TestBench bench = new TestBench();

    @Override
    public void init(){
        bench.init(hardwareMap);

    }
    double speedIncrement = 0;
    @Override
    public void loop(){
        DcMotorEx motor = bench.getMotor3();
        telemetry.addData("Motor 3 Type",motor.getMotorType());
//        bench.setMotorSpeed(1500);
//        bench.setMotorSpeed(1500);

        telemetry.addData("TicksPerRev",bench.ticksPerRev);
        telemetry.addData("Motor Revs",bench.getMotorRev());
        telemetry.addData("a",gamepad1.a);
        telemetry.addData("b",gamepad1.b);
        telemetry.addData("y",gamepad1.y);
        telemetry.addData("power",speedIncrement);
        telemetry.addData("velocity",motor.getVelocity());
        telemetry.addData("rpm 19.2",motor.getVelocity()*60/28/19.2);
        telemetry.addData("rpm1:1 6000",motor.getVelocity()*60/28);
//        updateTelemetry("rpm",motor.);
        if (gamepad1.bWasPressed()){
            speedIncrement += .1;
        }
        if (gamepad1.yWasPressed()){
            speedIncrement -= .1;
        }
        if (gamepad1.a) {
           bench.setMotorSpeed(0);
        } else {
            bench.setMotorSpeed(speedIncrement);

        }
    }
}
