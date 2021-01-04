package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;

public class ForkliftSubsystem implements ISubsystem<ForkliftStateMachine, ForkliftStateMachine.State> {
    private static ForkliftStateMachine forkliftStateMachine;
    private DcMotor forkliftMotor;

    public ForkliftSubsystem(){
        setForkliftStateMachine(new ForkliftStateMachine());
        setMotor(forkliftMotor);
    }

    @Override
    public ForkliftStateMachine getStateMachine() {
        return forkliftStateMachine;
    }

    @Override
    public ForkliftStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Forklift Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getForkliftMotor().setPower(getState().getPosition());
    }

    public DcMotor getForkliftMotor(){
        return forkliftMotor;
    }

    public static void setForkliftStateMachine(ForkliftStateMachine forkliftStateMachine){
        ForkliftSubsystem.forkliftStateMachine = forkliftStateMachine;
    }

    public void setMotor(DcMotor motor){
        this.forkliftMotor = motor;
    }

}
