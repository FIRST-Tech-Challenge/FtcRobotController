package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class MoveSliderCommand extends SounderBotCommandBase {

    DeliverySlider slider;
    Telemetry telemetry;

    double target;

    Motor motor;

    SonicPIDController pidController;

    double position;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target) {
        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        addRequirements(slider);
    }

    @Override
    public void execute() {
        position = motor.encoder.getPosition();
        double power = pidController.calculatePIDAlgorithm(target - position);

        if(isTargetReached()) {
            motor.set(0);
            finished.set(true);
        } else {
            double minPower = .2;

            if (Math.abs(power) < minPower) {

                power = minPower * Math.abs(power) / power;
            }

            motor.set(power);
        }
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 40;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }
}
