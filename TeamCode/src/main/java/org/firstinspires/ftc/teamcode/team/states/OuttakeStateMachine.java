package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class OuttakeStateMachine extends TimedState<OuttakeStateMachine.State> {
    public OuttakeStateMachine(){
        super(State.INIT);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Outtake State Machine";
    }

    public enum State implements Namable{
        PICKUP("PICKUP", 0.025d), INIT("Init", 0.5d), RELEASE("RELEASE", 0.975d); //KHALID TO-DO

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