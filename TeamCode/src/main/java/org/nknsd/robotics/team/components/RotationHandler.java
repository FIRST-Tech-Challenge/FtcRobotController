package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.concurrent.TimeUnit;

public class RotationHandler implements NKNComponent {

    PotentiometerHandler potHandler;
    private final String motorName;
    private DcMotor motor;
    public RotationPositions targetRotationPosition;

    final double threshold;
    final double P_CONSTANT;
    double diff;
    long targetTime = 0;
    double current;
    private ExtensionHandler extensionHandler;

    public enum RotationPositions {
        PICKUP(1.3),
        PREPICKUP(1.6),
        HIGH(2.4),
        RESTING(2.6);

        public final double target;
        RotationPositions(double target) {
            this.target = target;
        }
    }


    public RotationHandler(String motorName, double threshold, double P_CONSTANT){
        this.motorName = motorName;
        this.threshold = threshold;
        this.P_CONSTANT = P_CONSTANT;
    }

    public void link(PotentiometerHandler potHandler, ExtensionHandler extensionHandler){
        this.potHandler = potHandler;
        this.extensionHandler = extensionHandler;
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
        telemetry.addData("armTarget", targetRotationPosition.target);
        telemetry.addData("armDifference", diff);
        telemetry.addData("Motor Power", motor.getPower());

    }

    public void setTargetRotationPosition(RotationPositions targetRotationPosition){
        if (extensionHandler.targetPosition() == ExtensionHandler.ExtensionPositions.RESTING) {this.targetRotationPosition = targetRotationPosition;}
    }

    private double controlLoop(double current){
        diff = (targetRotationPosition.target - current);
        if (Math.abs(diff) <= threshold) {
            return 0;
        }
        return (diff * P_CONSTANT);
    }
}
