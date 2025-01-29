package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class MovePivotCommand extends SounderBotCommandBase{

    DeliveryPivot pivot;
    Telemetry telemetry;
    double target;
    double position, previousPosition = 1000000;
    Motor motor;
    SonicPIDFController pidController;
    int startDelayMs = 0;
    int endDelayMs = 0;

    double maxPower = 1;

    boolean startDelayExecuted = false;

    public MovePivotCommand(DeliveryPivot pivot, Telemetry telemetry, double target) {
        this(pivot,telemetry, target, 0, 0, 1);
    }

    public MovePivotCommand(DeliveryPivot pivot, Telemetry telemetry, double target, int startDelayMs, int endDelayMs, double maxPower) {
        super(3000);

        this.pivot = pivot;
        this.telemetry = telemetry;
        this.target = target;
        this.motor = pivot.getMotor();
        this.pidController = pivot.getPidController();
        this.startDelayMs = startDelayMs;
        this.endDelayMs = endDelayMs;
        this.maxPower = maxPower;

        addRequirements(pivot);
    }

    @Override
    protected boolean isTargetReached() {
        if (pivot.GetDepth() < 27 && target >  500) {
            target = Math.min(target, position + 100);
        }
        return Math.abs(target - position) < 30;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }

    @Override
    public void initialize() {
        super.initialize();
        startDelayExecuted = false;
    }

    @Override
    public void doExecute() {
        if (!startDelayExecuted && startDelayMs > 0) {
            sleep(startDelayMs);
            startDelayExecuted = true;
        }
        position = motor.encoder.getPosition();
        double power = pidController.calculatePIDAlgorithm(target - position);
        if (isTargetReached()) {
            motor.set(0);
            finished = true;
            sleep(endDelayMs);
        } else {
            double minPower = .2;

            if (Math.abs(power) < minPower) {
                //telemetry.addData("minPower", true);

                power = minPower * Math.signum(power);
            } else{

                power = Math.min(power, 1) * maxPower;
            }

            motor.set(power);
        }

        previousPosition = position;
    }
}
