package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.utility.StateMachine;

public class RaiseLiftState extends StateMachine.State {
    private final BallLiftHandler ballLiftHandler;
    final double delayMS = 100;

    public RaiseLiftState(BallLiftHandler ballLiftHandler) {
        this.ballLiftHandler = ballLiftHandler;
    }

    @Override
    protected void run(ElapsedTime runtime, Telemetry telemetry) {
        if(runtime.milliseconds() > delayMS + startTimeMS){
            ballLiftHandler.raiseLift();
            StateMachine.INSTANCE.stopAnonymous(this);
        }
    }

    @Override
    protected void started() {
        ballLiftHandler.lowerGate();
    }

    @Override
    protected void stopped() {

    }
}
