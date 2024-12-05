package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MoveSliderCommand extends SounderBotCommandBase {

    public static class EndAction {
        public static final double DEFAULT_END_POWER = .2;
        public double endPowerWithoutSign;
        Supplier<Boolean> stopMotorSignalProvider;

        public EndAction(double endPowerWithoutSign, Supplier<Boolean> stopMotorSignalProvider) {
            this.endPowerWithoutSign = endPowerWithoutSign;
            this.stopMotorSignalProvider = stopMotorSignalProvider;
        }

        public EndAction(Supplier<Boolean> stopMotorSignalProvider) {
            this(DEFAULT_END_POWER, stopMotorSignalProvider);
        }
    }
    DeliverySlider slider;
    Telemetry telemetry;

    double target;

    Motor motor;

    SonicPIDFController pidController;

    double position;

    EndAction endAction;

    int holdPosition = Integer.MIN_VALUE;

    double holdingPower = 0;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target) {
        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        addRequirements(slider);
    }

    public MoveSliderCommand withEndAction(EndAction endAction) {
        this.endAction = endAction;
        return this;
    }

    @Override
    public void doExecute() {
        position = motor.encoder.getPosition();
        double power = pidController.calculatePIDAlgorithm(target - position);

        if(isTargetReached() || holdPosition != Integer.MIN_VALUE) {
            if (endAction == null) {
                motor.set(0);
                finished = true;
                telemetry.addLine("Done");
            } else {
                if (holdPosition == Integer.MIN_VALUE) {
                    holdPosition = motor.getCurrentPosition();
                    holdingPower = endAction.endPowerWithoutSign * Math.signum(power);
                }

                if (endAction.stopMotorSignalProvider.get()) {
                    motor.set(0);
                    finished = true;
                } else {
                    telemetry.addLine("holding slider...");
                    if (Math.abs(holdPosition - motor.getCurrentPosition()) < 10) {
                        motor.set(0);
                    } else {
                        motor.set(holdingPower);
                    }
                }

            }

        } else {
            double minPower = .2;

            if (Math.abs(power) < minPower) {

                power = minPower * Math.abs(power) / power;
            }

            telemetry.addData("slider", position);
            telemetry.addData("target", target);
            telemetry.addData("power", power);

            motor.set(power);
        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 70;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }
}
