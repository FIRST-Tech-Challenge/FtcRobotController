package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.concurrent.TimeUnit;

public class ArmRotator implements NKNComponent {

    PotentiometerHandler potHandler;
    private final String motorName;
    private DcMotor motor;
    double target;

    final double threshold;
    final double K;
    double diff;
    long targetTime = 0;
    double current;


    public ArmRotator(String motorName, double threshold, double K){
        this.motorName = motorName;
        this.threshold = threshold;
        this.K = K;
    }

    public void link(PotentiometerHandler potHandler){
        this.potHandler = potHandler;
    }


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(motorName);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "ArmRotator";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        long currentTime = runtime.time(TimeUnit.MILLISECONDS);
        if(currentTime >= targetTime) {
            current = potHandler.getPotVoltage();
            double armPower = controlLoop(current);

            targetTime = currentTime+1;
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("armPosition",target);
        telemetry.addData("armTarget",current);
        telemetry.addData("armDifference", diff);

    }
    public void setTarget(double target){
        this.target = target;
    }
    private double controlLoop(double current){
        diff = (target - current);
        if (Math.abs(diff) <= threshold) {
            return 0;
        }
        return (diff * K);
    }
}
