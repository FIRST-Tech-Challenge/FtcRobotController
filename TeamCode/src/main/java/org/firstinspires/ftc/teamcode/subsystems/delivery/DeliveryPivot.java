package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class DeliveryPivot extends SonicSubsystemBase {

    private Motor motor;

    private Servo stopper;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private int StartPositionFromCalibration = 1590;

    public static int DeliveryPositionFromStart = 1500; //CHANGED

    public static int IntakePositionFromStart = -920;

    private int SampleIntakePositionFromStart = -1490;

    private int SliderCheckLimit = 625;

    public static int StartPositionFromStart = 0;


    private boolean isTeleop = true;

    private int currentTarget = 0;

    SonicPIDFController pidController;

    public static double recordedPosition;

    RollingIntake rollingIntake;

    public DeliveryPivot(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, RollingIntake rollingIntake) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliveryPivot");
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.motor.encoder.reset();

        this.stopper  = hardwareMap.get(Servo.class,"Stopper");
        this.stopper.setPosition(1);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;
        this.rollingIntake = rollingIntake;

        this.recordedPosition = 0;

        pidController = new SonicPIDFController(0.005, 0, 0.000);
    }

    boolean isStopperClosed = false;

    public void ToggleStopper() {
        if(isStopperClosed) {
            CloseStopper();
        } else {
            OpenStopper();
        }

        isStopperClosed = !isStopperClosed;
    }

    private void CloseStopper() {
        this.stopper.setPosition(0.25);
    }

    private void OpenStopper() {
        this.stopper.setPosition(0.875);
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
        motor.set(-.5);
    }

    public void RotateTowardsDeliverySlowly() {
        SetTelop();
        motor.set(.5);
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
        this.currentTarget =100;
    }

    double previousPosition = -1000000.0;

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        boolean addTelemetry = true;

        if(addTelemetry) {
            telemetry.addData("pivot target", currentTarget);
            telemetry.addData("pivot current", position);
        }

        if(position < DeliveryPositionFromStart - 300) {
            OpenStopper();
            rollingIntake.SetInDeliveryPositionn(false);
        }
        else {
            CloseStopper();
            rollingIntake.SetInDeliveryPositionn(true);
        }

        this.previousPosition = position;

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


    public Motor getMotor() {
        return motor;
    }

    public SonicPIDFController getPidController() {
        return pidController;
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }
}