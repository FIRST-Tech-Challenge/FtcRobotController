package org.firstinspires.ftc.teamcode.subsystems.delivery;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.function.Supplier;

@Config
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

    public static int BasketDeliveryPosition = 900;

    public static int MaxExtension = 910;

    public static int CollapsedPosition = -10;

    public static int StartPosition = 125;

    public static int SpecimenPosition = 295;

    public static int ExtendLimit = 420;

    private int currentTarget = 0;

    public static double pid_p = 0.0045;
    public static double pid_i = 0;
    public static double pid_d = 0;
    public static double pid_f = 0.02;

    public static SonicPIDFController pidController = new SonicPIDFController(pid_p, pid_i, pid_d, pid_f);

    private boolean isTeleop = true;

    public static double recordedPosition;

    private Supplier<Boolean> pivotLowEnoughSupplier;

    public static final double MAX_POWER = 1;

    private DeliveryPivot pivot;

    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, DeliveryPivot pivot) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "Slider1");
        this.motor2  = new Motor(hardwareMap, "Slider2");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;
        this.pivot = pivot;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDFController(pid_p, pid_i, pid_d, pid_f);
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
        currentTarget = SpecimenPosition;
    }

    public void MoveToDeliverySamplePosition() {
        SetAuto();
        currentTarget = BasketDeliveryPosition - 50;
    }

    public void MoveToMaxExtension() {

        double position = motor.encoder.getPosition();

        if(position > 700) {
            SetAuto();
            currentTarget = MaxExtension;
        }
    }

    public void MoveToCollapsedPosition() {
        currentTarget = CollapsedPosition;
    }

    public void MoveToCollapsedPositionForSpecimen() {
        isTeleop = false;
        currentTarget = CollapsedPosition;
    }

    public void MoveToSpecimenPosition() {
        isTeleop = false;
        currentTarget = SpecimenPosition;
    }

    @Override
    public void periodic() {
        super.periodic();

        double position = motor.encoder.getPosition();
        recordedPosition = position;
        Log.i("armControl", "slider position = " + position + ", action: " + (motor.get() > 0 ? "extend" : (motor.get() < 0 ? "Collapse" : "Stop")) );

        boolean addTelemetry = true;
        double expectedPivotAngle = Math.toDegrees(Math.asin(7.25 / (14.5 + position * 0.0329)));
        double sliderLength = position * 0.0329;
        int extraTicks = (int)((30 - expectedPivotAngle) * -3);

        if(addTelemetry) {
            telemetry.addData("slider target", currentTarget);
            telemetry.addData("slider current", position);
            telemetry.addData("slider length", sliderLength);
            telemetry.addData("expected pivot angle", expectedPivotAngle);

            telemetry.addData("currrent pivot position", pivot.getPosition());
            telemetry.addData("new pivot position", pivot.getPosition() + extraTicks);
            telemetry.update();
        }

        if(!isTeleop) {

            double power = pidController.calculatePIDAlgorithm(currentTarget - position);
            telemetry.addData("slider pid output", power);
            if(Math.abs(currentTarget - position) < 30) {
                motor.set(0);
                motor2.set(0);
            }
            else {
                double minPower = 0;

                if(Math.abs(power) < minPower) {
                    power = minPower * Math.abs(power) / power;
                }

                if(Math.abs(power) > 1) {
                    power = Math.signum(power) * MAX_POWER;
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

            if(addTelemetry) {
                telemetry.addData("slider", position);
                //telemetry.addData("pivot supplier", pivotLowEnoughSupplier.get());
                telemetry.addData("motor", motor.get());
            }


            if (pivotLowEnoughSupplier != null
                    && pivotLowEnoughSupplier.get()
                    && Math.abs(motor.get()) > 0) {
                if(position > ExtendLimit) {

                    motor.stopMotor();
                    motor2.stopMotor();
                    MoveToValidPosition();
                } else {
                    int newPivotPosition = pivot.getPosition();

                    if(sliderLength > 14) {
                        newPivotPosition = -1664;

                    } else if (sliderLength > 12.5) {
                        newPivotPosition = -1632;

                    } else if (sliderLength > 10) {
                        newPivotPosition = -1605;

                    } else if (sliderLength > 7.5) {
                        newPivotPosition = -1580;

                    } else if (sliderLength > 5) {
                        newPivotPosition = -1481;

                    } else if (sliderLength > 2.5) {
                        newPivotPosition = -1413;
                    } else {
                        newPivotPosition = -1300;
                    }

                    pivot.SetTargetToCurrent(newPivotPosition);
                }

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
        if (pidController.getKp() != pid_p ||
                pidController.getKi() != pid_i ||
                pidController.getKd() != pid_d ||
                pidController.getKf() != pid_f) {
            pidController = new SonicPIDFController(pid_p, pid_i, pid_d, pid_f);
        }
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