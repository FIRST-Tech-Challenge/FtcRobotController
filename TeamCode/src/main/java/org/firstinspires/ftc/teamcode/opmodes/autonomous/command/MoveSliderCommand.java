package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MoveSliderCommand extends SounderBotCommandBase {

    private static final String LOG_TAG = MoveSliderCommand.class.getSimpleName();
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

    boolean resetEncoder = false;

    DeliverySlider.Direction direction;

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, DeliverySlider.Direction direction) {
        this(slider, telemetry, target, false, direction);
    }

    public MoveSliderCommand(DeliverySlider slider, Telemetry telemetry, double target, boolean resetEncoder, DeliverySlider.Direction direction) {
        this.slider = slider;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = slider.getMotor();
        this.pidController = slider.getPidController();
        this.resetEncoder = resetEncoder;
        this.direction = direction;

        Log.i(LOG_TAG, "Target set to: " + target);
        addRequirements(slider);
    }

    public MoveSliderCommand withEndAction(EndAction endAction) {
        this.endAction = endAction;
        return this;
    }

    @Override
    public void doExecute() {
        position = motor.encoder.getPosition();
        Log.i(LOG_TAG, String.format("Current position = %f, direction = %s", position, direction.name()));
        double power = pidController.calculatePIDAlgorithm(target - position);

        if(isTargetReached() || holdPosition != Integer.MIN_VALUE) {
            Log.i(LOG_TAG, "target reached with direction " + direction.name());
            if (endAction == null) {
                Log.i(LOG_TAG, "no end action");
                end(false);
                finished = true;
                telemetry.addLine("Done");

                if(resetEncoder) {
                    this.slider.ResetEncoder();

                }
            } else {
                if (holdPosition == Integer.MIN_VALUE) {
                    holdPosition = motor.getCurrentPosition();
                    holdingPower = endAction.endPowerWithoutSign * Math.signum(power);
                }

                if (endAction.stopMotorSignalProvider.get()) {
                    slider.setMotors(0);
                    finished = true;
                } else {
                    telemetry.addLine("holding slider...");
                    if (Math.abs(holdPosition - motor.getCurrentPosition()) < 50) {
                        slider.setMotors(0);
                    } else {
                        slider.setMotors(holdingPower);
                    }
                }

            }

        } else {
            double minPower = .2;

            power = Math.max(minPower, Math.abs(power)) * direction.directionFactor;

            telemetry.addData("slider", position);
            telemetry.addData("target", target);
            telemetry.addData("power", power);


            Log.i(LOG_TAG, "Power to motor: " + power);
            slider.setMotors(power);
        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(target - position) < 100;
    }

    @Override
    public void end(boolean interrupted) {
        if (DeliverySlider.Direction.EXPANDING == direction) {
            slider.setMotors(-.1);
        } else {
            slider.setMotors(0);
        }
    }
}
