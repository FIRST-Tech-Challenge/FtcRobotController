package org.nknsd.teamcode.autoStates;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.drivers.MixedInputWheelDriver;
import org.nknsd.teamcode.components.handlers.BallLiftHandler;
import org.nknsd.teamcode.components.handlers.LowerLiftState;
import org.nknsd.teamcode.components.handlers.RaiseLiftState;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.utility.StateMachine;

public class AutoStupidLaunchState extends StateMachine.State{
    private final BallLiftHandler ballLiftHandler;
    private final PowerInputMixer powerInputMixer;

    public AutoStupidLaunchState(BallLiftHandler ballLiftHandler, PowerInputMixer powerInputMixer) {
        this.ballLiftHandler = ballLiftHandler;
        this.powerInputMixer = powerInputMixer;
    }

    int counter = 0;

    @Override
    protected void run(ElapsedTime runtime, Telemetry telemetry) {
        if(counter == 0 && runtime.milliseconds() > (1000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new RaiseLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 1 && runtime.milliseconds() > (2000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new LowerLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 2 && runtime.milliseconds() > (3000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new RaiseLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 3 && runtime.milliseconds() > (4000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new LowerLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 4 && runtime.milliseconds() > (5000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new RaiseLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 5 && runtime.milliseconds() > (6000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new LowerLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 6 && runtime.milliseconds() > (7000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new RaiseLiftState(ballLiftHandler));
            counter++;
        }
        if(counter == 7 && runtime.milliseconds() > (8000 + startTimeMS)){
            StateMachine.INSTANCE.startAnonymous(new LowerLiftState(ballLiftHandler));
            counter++;
        }
        if(runtime.milliseconds() > 10000 + startTimeMS){
            StateMachine.INSTANCE.stopAnonymous(this);
        }

    }

    @Override
    protected void started() {

    }

    @Override
    protected void stopped() {
        StateMachine.INSTANCE.startAnonymous(new AutoStupidForwardState(powerInputMixer));
    }
}
