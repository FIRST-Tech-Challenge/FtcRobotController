package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;
import com.technototes.logger.Stated;

public class IndexSubsystem extends ServoSubsystem implements Stated<Integer> {

    public Servo pivot, arm;


    public enum IndexState{
        EMPTY(0), ONE_RING(1), TWO_RINGS(2), FULL(3);
        public int numRings;
        IndexState(int rings){
            numRings = rings;
        }
        public int getNumRings(){
            return numRings;
        }

    }

    public IndexState indexState;

    public IndexSubsystem(Servo p, Servo a){
        super(p, a);
        pivot = p;
        arm = a;
        indexState = IndexState.EMPTY;
    }

    public void raiseToShooter(){
        pivot.setPosition(0.62);
    }
    public void lowerToIntake(){
        pivot.setPosition(0.4);
    }

    public void extendArm(){
        arm.setPosition(0.45);
    }

    public void retractArm(){
        arm.setPosition(0.2);
    }
    public int getNumRings(){
        return indexState.getNumRings();
    }


    @Override
    public Integer getState() {
        return getNumRings();
    }

    public boolean isFull() {
        return getNumRings()==3;
    }

}
