package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.EndGameExtensionStateMachine;

public class EndGameExtensionSubsystem implements ISubsystem<EndGameExtensionStateMachine, EndGameExtensionStateMachine.State> {
    private static EndGameExtensionStateMachine endGameExtensionStateMachine;

    private RevServo endGameExtensionBlocker;

    public EndGameExtensionSubsystem(RevServo endGameExtensionBlocker) {
        setEndGameExtensionStateMachine(new EndGameExtensionStateMachine());
        setEndGameExtensionBlocker(endGameExtensionBlocker);
    }

    @Override
    public EndGameExtensionStateMachine getStateMachine() {
        return endGameExtensionStateMachine;
    }

    @Override
    public EndGameExtensionStateMachine.State getState() {
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
        return "End Game Extension Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getEndGameExtensionBlocker().setPosition(getState().getPosition());
    }

    public static void setEndGameExtensionStateMachine(EndGameExtensionStateMachine endGameExtensionStateMachine) {
        EndGameExtensionSubsystem.endGameExtensionStateMachine = endGameExtensionStateMachine;
    }

    public RevServo getEndGameExtensionBlocker() {
        return endGameExtensionBlocker;
    }

    public void setEndGameExtensionBlocker(RevServo endGameExtensionBlocker) {
        this.endGameExtensionBlocker = endGameExtensionBlocker;
    }
}
