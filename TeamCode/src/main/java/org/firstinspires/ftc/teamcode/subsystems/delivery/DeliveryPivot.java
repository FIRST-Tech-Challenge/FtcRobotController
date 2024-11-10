package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

import java.util.Set;

public class DeliveryPivot extends SonicSubsystemBase {

    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private int StartPositionFromCalibration = 1975 - 310;

    private int DeliveryPositionFromStart = 400 + 310;

    private int IntakePositionFromStart = -1500 + 310;

    private int SampleIntakePositionFromStart = -1800 + 310;

    private int SliderCheckLimit = -625;

    private int StartPositionFromStart = 0;


    private boolean isTeleop = true;

    private int currentTarget = 0;

    SonicPIDController pidController;

    private double recordedPosition;

    public DeliveryPivot(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliveryPivot");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.motor.encoder.reset();

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.recordedPosition = 0;

        pidController = new SonicPIDController(0.005, 0, 0.000);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void RotateTowardsIntake() {
        SetTelop();
        motor.set(-1);
    }

    public void RotateTowardsDelivery() {
        SetTelop();
        motor.set(1);
    }

    public void RotateTowardsIntakeSlowly() {
        SetTelop();
        motor.set(-.33);
    }

    public void RotateTowardsDeliverySlowly() {
        SetTelop();
        motor.set(.33);
    }

    public void HoldArm() {
        SetTelop();
        motor.set(0);
    }

    public void Calibrate() {
        SetAuto();
        this.motor.encoder.reset();
        this.currentTarget = StartPositionFromCalibration;
    }

    public void AutoToDelivery() {
        SetAuto();
        this.currentTarget = DeliveryPositionFromStart;
    }

    public void AutoToIntake() {
        SetAuto();
        this.currentTarget = IntakePositionFromStart;
    }

    public void AutoToStart() {
        SetAuto();
        this.currentTarget = 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        //telemetry.addData("target", currentTarget);
        //telemetry.addData("current", position);
        //telemetry.addData("telop", isTeleop);

        if(!isTeleop) {
            double power = pidController.calculatePIDAlgorithm(currentTarget - position);
            //telemetry.addData("power", power);


            if(Math.abs(currentTarget - position) < 15) {
                //telemetry.addData("done", true);
                motor.set(0);
            }
            else {
                double minPower = .1;

                if(Math.abs(power) < minPower) {
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }
        } else {
            if (motor.get() != 0 && position != 0) {
                recordedPosition = position;
            }
            double currentPower = motor.get();
            String action = currentPower > 0 ? "up" : (currentPower < 0 ? "down" : "stop");
            Log.i("armControl", "pivot position = " + position + ", current action: " + action + ", too low? " + (lowEnoughToLimitSlider() ? "yes" : "no"));
//            if (!isCurrentPositionWithinLimit()) {
//                motor.stopMotor();
//            }
        }

        //telemetry.update();
    }

    public void MoveToIntakeInAuto() {
        MoveToPositionInAuto(IntakePositionFromStart);
    }

    public void MoveToDeliveryInAuto() {
        MoveToPositionInAuto(DeliveryPositionFromStart);
    }

    public void MoveToStartInAuto() {
        MoveToPositionInAuto(StartPositionFromStart);
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
                    //telemetry.addData("minPower", true);

                    power = minPower * Math.abs(power) / power;
                }

                motor.set(power);
            }

        }
    }

    public void MoveToIntakeSampleInAuto() {
        double position = motor.encoder.getPosition();

        motor.set(-0.5);
        while(position > SampleIntakePositionFromStart) {
            position = motor.encoder.getPosition();
        }

        motor.set(0);
    }

    public boolean lowEnoughToLimitSlider() {
        return motor.encoder.getPosition() < SliderCheckLimit;
    }

}