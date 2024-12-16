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

    public enum Direction {
        EXPANDING (-1),
        COLLAPSE (1);

        public final int directionFactor;

        Direction(int directionFactor) {
            this.directionFactor = directionFactor;
        }
    }

    private Motor motor;
    private Motor motor2;


    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public static int BasketDeliveryPosition = 863;
    public static int CollapsedPosition = -10;

    public static int StartPosition = 400;

    private int ExtendLimit = 880;

    private int currentTarget = 0;

    SonicPIDFController pidController;

    private boolean isTeleop = true;

    public static double recordedPosition;

    private Supplier<Boolean> pivotLowEnoughSupplier;

    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "Slider1");
        this.motor2  = new Motor(hardwareMap, "Slider2");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDFController(0.005, 0, 0, 0.05);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void collapse() {
        SetTelop();
        motor.set(.6);
        motor2.set(-.6);

    }

    public void expand() {
        SetTelop();
        if (pivotLowEnoughSupplier != null
                && pivotLowEnoughSupplier.get()) {
            motor.set(-.5);
            motor2.set(.5);
        } else {
            motor.set(-1);
            motor2.set(1);
        }
    }

    public void ExpandSlowly() {
        SetTelop();
        motor.set(-.3);
    }

    public void CollapseSlowly() {
        SetTelop();
        motor.set(.3);
    }

    public void Hold() {
        SetTelop();
        motor.set(0);
        motor2.set(0);
    }

    public void MoveToValidPosition() {
        SetAuto();
        currentTarget = ExtendLimit + 50;
    }

    public void MoveToDeliverySpecimanPosition() {
        SetAuto();
        currentTarget = StartPosition;
    }

    public void MoveToDeliverySamplePosition() {
        SetAuto();
        currentTarget = BasketDeliveryPosition - 50;
    }

    public void MoveToCollapsedPosition() {
        currentTarget = CollapsedPosition;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        recordedPosition = position;
        Log.i("armControl", "slider position = " + position + ", action: " + (motor.get() > 0 ? "extend" : (motor.get() < 0 ? "Collapse" : "Stop")) );

        boolean addTelemetry = false;
        if(addTelemetry) {
            telemetry.addData("slider target", currentTarget);
            telemetry.addData("slider current", position);
            telemetry.update();
        }

        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);

            if(Math.abs(currentTarget - position) < 50) {
                motor.set(0);
                motor2.set(0);
            }
            else {
                double minPower = 0;

                if(Math.abs(power) < minPower) {
                    power = minPower * Math.abs(power) / power;
                }

                if(Math.abs(power) > 1) {
                    power = Math.signum(power);
                }

                double slowFactor = 1;

                if(power < 0) {
                    slowFactor = 0.6;
                }

                power = power * slowFactor;

                motor.set(-power);
                motor2.set(power);
            }
        } else {
            //Log.i("armControl", "low enough? " + pivotLowEnoughSupplier == null ? "null" : (pivotLowEnoughSupplier.get() ? "yes" : "no"));

            telemetry.addData("slider", position);
            //telemetry.addData("pivot supplier", pivotLowEnoughSupplier.get());
            telemetry.addData("motor", motor.get());
            telemetry.update();


            if (pivotLowEnoughSupplier != null
                    && pivotLowEnoughSupplier.get()
                    && Math.abs(motor.get()) > 0
                    && position > ExtendLimit) {

                    motor.stopMotor();
                    motor2.stopMotor();
                    MoveToValidPosition();
            }

            if (position > BasketDeliveryPosition &&
                     Math.abs(motor.get()) > 0){
                motor.stopMotor();
                motor2.stopMotor();
                currentTarget = BasketDeliveryPosition;
            }
        }

        //telemetry.update();
    }

    public void ExtendMaxInAuto() {
        MoveToPositionInAuto(BasketDeliveryPosition + 100);
    }

    public void ResetEncoder() {
        this.motor.encoder.reset();
        this.motor2.encoder.reset();
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

    public void setMotors(double power) {
        motor.set(power);
        motor2.set(-power);
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }
}