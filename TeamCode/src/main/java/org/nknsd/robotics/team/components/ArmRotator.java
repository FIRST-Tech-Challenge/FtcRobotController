package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    final double P_CONSTANT;
    final double I_CONSTANT;
    double diff;
    long targetTime = 0;
    double current;
    double resError;


    public ArmRotator(String motorName, double threshold, double P_CONSTANT, double I_CONSTANT){
        this.motorName = motorName;
        this.threshold = threshold;
        this.P_CONSTANT = P_CONSTANT;
        this.I_CONSTANT = I_CONSTANT;
    }

    public void link(PotentiometerHandler potHandler){
        this.potHandler = potHandler;
    }


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            motor.setPower(armPower);
            targetTime = currentTime+1;
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("armPosition",current);
        telemetry.addData("armTarget",target);
        telemetry.addData("armDifference", diff);
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("residualError", resError);

    }
    public void setTarget(double target){
        this.target = target;
    }
    private double controlLoop(double current){
        diff = (target - current);
        resError += diff;
        if (Math.abs(diff) <= threshold) {
            resError = 0;
            return 0;
        }
        return ((diff * P_CONSTANT) + (resError * I_CONSTANT));
    }
}
