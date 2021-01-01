package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

public class ShooterSubsystem implements ISubsystem<ShooterStateMachine, ShooterStateMachine.State> {

    private static ShooterStateMachine shooterStateMachine;
    private RevMotor ShooterWheels;

    public ShooterSubsystem(RevMotor shooterMotor){
        setShooterStateMachine(new ShooterStateMachine());
        setShooterWheels(shooterMotor);
    }

    @Override
    public ShooterStateMachine getStateMachine() {
        return shooterStateMachine;
    }

    @Override
    public ShooterStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getIntakeWheels().setPower(0d);

    }

    @Override
    public String getName() {
        return "Shooter Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getIntakeWheels().setPower(getState().getPower());
    }

    private static void setShooterStateMachine(ShooterStateMachine ShooterStateMachine){
        ShooterSubsystem.shooterStateMachine = ShooterStateMachine;
    }

    private void setShooterWheels(RevMotor intakeMotor){
        this.ShooterWheels = intakeMotor;
    }

    private RevMotor getIntakeWheels(){
        return ShooterWheels;
    }
}
