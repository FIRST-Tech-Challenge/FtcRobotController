package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.function.Supplier;

public class DeliverySlider extends SonicSubsystemBase {

    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public static int BasketDeliveryPosition = -3300;
    public static int CollapsedPosition = -90;

    private int ExtendLimit = -1100;

    private int currentTarget = 0;

    SonicPIDFController pidController;

    private boolean isTeleop = true;

    private Supplier<Boolean> pivotLowEnoughSupplier;

    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliverySlider");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDFController(0.08, 0, 0);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void Expand() {
        SetTelop();
        motor.set(1);
    }

    public void Collapse() {
        SetTelop();
        motor.set(-1);
    }

    public void Hold() {
        SetTelop();
        motor.set(0);
    }

    public void MoveToValidPosition() {
        SetAuto();
        currentTarget = ExtendLimit + 50;
    }


    public void MoveToDeliveryPosition() {
        SetAuto();
        currentTarget = BasketDeliveryPosition;
    }

    public void MoveToTransferPosition() {
        currentTarget = CollapsedPosition;
    }

    @Override
    public void periodic() {
        double position = motor.encoder.getPosition();
        Log.i("armControl", "slider position = " + position + ", action: " + (motor.get() > 0 ? "extend" : (motor.get() < 0 ? "Collapse" : "Stop")) );
        //telemetry.addData("target", currentTarget);
        //telemetry.addData("current", position);
        //telemetry.addData("telop", isTeleop);

        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);
            //telemetry.addData("power", power);


            if(Math.abs(currentTarget - position) < 40) {
                //telemetry.addData("done", true);
                motor.set(0);
            }
            else {
                double minPower = .2;

                if(Math.abs(power) < minPower) {
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        } else {
//            Log.i("armControl", "low enough? " + pivotLowEnoughSupplier == null ? "null" : (pivotLowEnoughSupplier.get() ? "yes" : "no"));
            if (pivotLowEnoughSupplier != null
                    && pivotLowEnoughSupplier.get()
                    && Math.abs(motor.get()) > 0
                    && position < ExtendLimit) {
                motor.stopMotor();
                MoveToValidPosition();

            }
        }

        //telemetry.update();
    }

    public void ExtendMaxInAuto() {
        MoveToPositionInAuto(BasketDeliveryPosition + 100);
    }

    public void CollapseMinInAuto() {
        MoveToPositionInAuto(CollapsedPosition);
    }

    public void MoveToPositionInAuto(double target) {
        while(true) {

            double position = motor.encoder.getPosition();

            double power = pidController.calculatePIDAlgorithm(target - position);

            if(Math.abs(target - position) < 40) {
                motor.set(0);
                break;
            }
            else {
                double minPower = .2;

                if (Math.abs(power) < minPower) {

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        }
    }

    public void setPivotLowEnoughSupplier(Supplier<Boolean> pivotLowEnoughSupplier) {
        this.pivotLowEnoughSupplier = pivotLowEnoughSupplier;
    }

    public boolean isMotorStopped() {
        return motor.get() == 0;
    }

    public Motor getMotor() {
        return motor;
    }

    public SonicPIDFController getPidController() {
        return pidController;
    }
}