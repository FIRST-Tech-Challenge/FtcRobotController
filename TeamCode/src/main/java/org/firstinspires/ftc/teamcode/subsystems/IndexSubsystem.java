package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.command.Command;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;
import com.technototes.logger.Stated;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmExtendCommand;
import org.firstinspires.ftc.teamcode.commands.index.ArmRetractCommand;

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
        pivot = p;
        arm = a;
    }

    public void raiseToShooter(){
        //TODO get position values
        pivot.setPosition(1);
    }
    public void lowerToIntake(){
        //TODO get position values
        pivot.setPosition(0);
    }

    public void extendArm(){
        //TODO get position values
        arm.setPosition(1);
    }

    public void retractArm(){
        //TODO get position values
        arm.setPosition(0);
    }
    public int getNumRings(){
        return indexState.getNumRings();
    }


    @Override
    public Integer getState() {
        return getNumRings();
    }


}
