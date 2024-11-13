package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.helperClasses.PIDModel;

import java.util.concurrent.TimeUnit;

public class RotationHandler implements NKNComponent {

    public static final int MAX_INDEX_OF_ROTATION_POSITIONS = 5;
    final double threshold = 0.03;

    final double errorCap = 10;
    final boolean enableErrorClear = true;
    private final String motorName = "motorArmRotate";
    public RotationPositions targetRotationPosition = RotationPositions.RESTING;
    PotentiometerHandler potHandler;
    private boolean isErrorPositive;
    double diff;
    long targetTime = 0;
    double current;
    double resError;
    private DcMotor motor;
    private ExtensionHandler extensionHandler;
    final private PIDModel normalPIDModel= new PIDModel(0.43,0.005,15) ;
    final private PIDModel extendedPIDModel = new PIDModel(0.6,0.007,30);

    public RotationHandler() {}

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public void link(PotentiometerHandler potHandler, ExtensionHandler extensionHandler) {
        this.potHandler = potHandler;
        this.extensionHandler = extensionHandler;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (potHandler.getPotVoltage() < RotationPositions.HIGH.target) { // Arm is already rotated out when initializing, so we can move to pickup position
            targetRotationPosition = RotationPositions.PICKUP;
        }
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "ArmRotator";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        long currentTime = runtime.time(TimeUnit.MILLISECONDS);
        if (currentTime >= targetTime) {
            current = potHandler.getPotVoltage();
            double armPower = controlLoop(current, runtime);

            motor.setPower(armPower);
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm Rot Position", current);
        telemetry.addData("Arm Rot Target", targetRotationPosition.target);
        telemetry.addData("Arm Rot Diff", diff);
        telemetry.addData("Arm Rot Motor Power", motor.getPower());
        telemetry.addData("Arm Rot Residual Err", resError);
    }

    public void setTargetRotationPosition(RotationPositions targetRotationPosition) {
        if (extensionHandler.targetPosition() == ExtensionHandler.ExtensionPositions.RESTING) {
            this.targetRotationPosition = targetRotationPosition;

        } else if (targetRotationPosition == RotationPositions.PICKUP && this.targetRotationPosition == RotationPositions.PREPICKUP) {
            this.targetRotationPosition = targetRotationPosition;

        } else if (targetRotationPosition == RotationPositions.PREPICKUP && this.targetRotationPosition == RotationPositions.PICKUP) {
            this.targetRotationPosition = targetRotationPosition;
        }
    }

    private boolean oppositeSigns(double one, double two) {
        return one * two < 0; // If the two have DIFFERENT signs, multiplying them will give us a negative number
    }

    private double controlLoop(double current, ElapsedTime runtime) {
        diff = (targetRotationPosition.target - current);
        if ((isErrorPositive && diff < 0) || (!isErrorPositive && diff > 0)){
            normalPIDModel.resetError();
            extendedPIDModel.resetError();
        }
        if (diff > 0){
            isErrorPositive = true;
        }else{
            isErrorPositive = false;
        }
        //calculates PID values
        if (Math.abs(diff) <= threshold) {
            return 0;
        }
        if(extensionHandler.targetPosition() != ExtensionHandler.ExtensionPositions.HIGH_BASKET){
            extendedPIDModel.resetError();
            return normalPIDModel.calculate(current, targetRotationPosition.target, runtime);
        } else {
            normalPIDModel.resetError();
            return extendedPIDModel.calculate(current, targetRotationPosition.target, runtime);
        }
    }

    public boolean isAtTargetPosition() {
        return Math.abs(targetRotationPosition.target - motor.getCurrentPosition()) <= threshold * 2;
    }

    public enum RotationPositions {
        PICKUP(0.63), PREPICKUP(0.92), HIGH(1.47), RESTING(2.2), SPECIMEN(2.26);

        public final double target;

        RotationPositions(double target) {
            this.target = target;
        }
    }
}
