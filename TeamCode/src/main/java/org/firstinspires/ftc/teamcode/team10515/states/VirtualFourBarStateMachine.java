package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class VirtualFourBarStateMachine extends TimedState<VirtualFourBarStateMachine.State> {
    public VirtualFourBarStateMachine(){
        super(State.INIT);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Dropper Left State Machine";
    }

    public enum State implements Namable{
        LEFT("LEFT", 0.025d), INIT("Init", 0.5d), RIGHT("RIGHT", 0.975d);

        private final String name;
        private final double position;

        State(final String name, final double position){
            this.name = name;
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        @Override
        public String getName(){
            return name;
        }
    }
}