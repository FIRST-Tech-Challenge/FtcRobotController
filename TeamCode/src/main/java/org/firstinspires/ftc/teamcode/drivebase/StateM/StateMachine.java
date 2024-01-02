package org.firstinspires.ftc.teamcode.drivebase.StateM;

/**
 * Created by michael on 10/2/18.
 */

public abstract class StateMachine<STATE_VAR extends Enum<STATE_VAR>>
{
    public STATE_VAR state;
    private long stateStartTime;

    public enum ReturnState
    {
        KEEP_RUNNING_ME,
        PROCEED
    }

    public void switchState(STATE_VAR state)
    {
        System.out.println("[" + getName() + "] Switching state to: " + state.toString());
        this.state = state;
        stateStartTime = System.currentTimeMillis();
    }

    public long getElapsedStateTime()
    {
        return System.currentTimeMillis() - stateStartTime;
    }

    public void nested(StateMachine stateMachine, STATE_VAR switchToWhenFinished)
    {
        if(stateMachine.runIteration() == ReturnState.PROCEED)
        {
            switchState(switchToWhenFinished);
        }
    }

    public abstract String getName();
    public abstract ReturnState runIteration();
    public abstract void reset();

}
