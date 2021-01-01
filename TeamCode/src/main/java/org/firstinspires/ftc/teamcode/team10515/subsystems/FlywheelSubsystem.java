package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.FlywheelStateMachine;

public class FlywheelSubsystem implements ISubsystem<FlywheelStateMachine, FlywheelStateMachine.State> {
    private static FlywheelStateMachine flywheelStateMachine;
    private RevMotor intakeWheels;
    //private RevMotor leftFlywheel;
    //private RevMotor rightFlywheel;

    public FlywheelSubsystem(RevMotor intakeMotor){//RevMotor leftFlywheel, RevMotor rightFlywheel) {
        setFlywheelStateMachine(new FlywheelStateMachine());
       setIntakeWheels(intakeMotor);
//        setLeftFlywheel(leftFlywheel);
//        setRightFlywheel(rightFlywheel);
    }

    @Override
    public FlywheelStateMachine getStateMachine() {
        return flywheelStateMachine;
    }

    @Override
    public FlywheelStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
//        getLeftFlywheel().setPower(0d);
//        getRightFlywheel().setPower(0d);
        getIntakeWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getIntakeWheels().setPower(getState().getPower());
//        getLeftFlywheel().setPower(getState().getPower());
//        getRightFlywheel().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setFlywheelStateMachine(FlywheelStateMachine flywheelStateMachine) {
        FlywheelSubsystem.flywheelStateMachine = flywheelStateMachine;
    }

//    private RevMotor getLeftFlywheel() {
//        return leftFlywheel;
//    }
//
//    private void setLeftFlywheel(RevMotor leftFlywheel) {
//        this.leftFlywheel = leftFlywheel;
//    }
//
//    private RevMotor getRightFlywheel() {
//        return rightFlywheel;
//    }
//
//    private void setRightFlywheel(RevMotor rightFlywheel) {
//        this.rightFlywheel = rightFlywheel;
//    }

    private void setIntakeWheels(RevMotor intakeMotor){
        this.intakeWheels = intakeMotor;
    }
    private RevMotor getIntakeWheels(){
        return intakeWheels;
    }
}
