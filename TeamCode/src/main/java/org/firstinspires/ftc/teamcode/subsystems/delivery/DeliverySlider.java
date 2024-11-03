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

    private int maxPosition = -3000;
    private int minPosition = -200;

    private int currentTarget = 0;


    SonicPIDController pidController;

    public DeliverySlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliverySlider");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();

        //MoveToTransferPosition();

        //pidController = new SonicPIDController(0.004, 0.0001, 0.0003);
    }

    public void Expand() {
        motor.set(.75);
    }

    public void Collapse() {
        motor.set(-.75);
    }

    public void Hold() {
        motor.set(0);
    }

    public void MoveToDeliveryPosition() {

        currentTarget = -3000;
    }

    public void MoveToTransferPosition() {
        currentTarget = -100;
    }

    @Override
    public void periodic() {
        super.periodic();

//        double position = motor.encoder.getPosition();
//
//        double power = pidController.calculatePIDAlgorithm(currentTarget - position);
//
//        telemetry.addData("target", position);
//        telemetry.addData("position", position);
//        telemetry.addData("power", power);
//        telemetry.update();
//
//        motor.set(power);
    }
}