package org.firstinspires.ftc.teamcode.subsystems.delivery;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class DeliverySlider extends SonicSubsystemBase {

    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private int BasketDeliveryPosition = -3300;
    private int CollapsedPosition = -100;

    private int currentTarget = 0;

    SonicPIDController pidController;

    private boolean isTeleop = true;

    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliverySlider");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        pidController = new SonicPIDController(0.005, 0, 0);
    }

    private void SetTelop() {
        this.isTeleop = true;
    }

    private void SetAuto() {
        this.isTeleop = false;
    }

    public void Expand() {
        SetTelop();
        motor.set(.75);
    }

    public void Collapse() {
        SetTelop();
        motor.set(-.75);
    }

    public void Hold() {
        SetTelop();
        motor.set(0);
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
        super.periodic();

        double position = motor.encoder.getPosition();
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
        }

        //telemetry.update();
    }
}