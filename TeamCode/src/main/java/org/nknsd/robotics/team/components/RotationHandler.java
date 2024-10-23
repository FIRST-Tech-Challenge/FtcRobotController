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
    public RotationPositions targetRotationPosition = RotationPositions.RESTING;

    final double threshold;
    final double P_CONSTANT;
    final double I_CONSTANT;
    final double errorCap;
    double diff;
    long targetTime = 0;
    double current;
    double resError;
    final boolean enableErrorClear;
    private ExtensionHandler extensionHandler;

    public enum RotationPositions {
        PICKUP(1.2),
        PREPICKUP(1.47),
        HIGH(2.4),
        RESTING(3.3);

        public final double target;
        RotationPositions(double target) {
            this.target = target;
        }
    }


    public RotationHandler(String motorName, double threshold, double P_CONSTANT, double I_CONSTANT, double errorCap, boolean enableErrorClear){
        this.motorName = motorName;
        this.threshold = threshold;
        this.P_CONSTANT = P_CONSTANT;
        this.I_CONSTANT = I_CONSTANT;
        this.errorCap = errorCap;
        this.enableErrorClear = enableErrorClear;
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
        telemetry.addData("Arm Rot Position", current);
        telemetry.addData("Arm Rot Target", targetRotationPosition.name());
        telemetry.addData("Arm Rot Diff", diff);
        telemetry.addData("Arm Rot Motor Power", motor.getPower());
        telemetry.addData("Arm Rot Residual Err", resError);
    }

    public void setTargetRotationPosition(RotationPositions targetRotationPosition){
        if (extensionHandler.targetPosition() == ExtensionHandler.ExtensionPositions.RESTING) {
            this.targetRotationPosition = targetRotationPosition;

        } else if (targetRotationPosition == RotationPositions.PICKUP && this.targetRotationPosition == RotationPositions.PREPICKUP) {
            this.targetRotationPosition = targetRotationPosition;

        } else if (targetRotationPosition == RotationPositions.PREPICKUP && this.targetRotationPosition == RotationPositions.PICKUP) {
            this.targetRotationPosition = targetRotationPosition;
        }
    }

    private boolean oppositeSigns(double one, double two){
        return one * two < 0; // If the two have DIFFERENT signs, multiplying them will give us a negative number
    }

    private double controlLoop(double current){
        diff = (targetRotationPosition.target - current);
        resError += diff;

        if (Math.abs(diff) <= threshold) {
            return 0;
        }

        if (resError > errorCap) {
            resError = errorCap;
        }

        if (oppositeSigns(diff, resError) && enableErrorClear){
            resError = diff;
        }

        return ((diff * P_CONSTANT) + (resError * I_CONSTANT));
    }
}
