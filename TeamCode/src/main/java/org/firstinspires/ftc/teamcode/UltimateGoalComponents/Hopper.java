package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import android.os.CpuUsageInfo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class Hopper extends RobotComponent {
    Servo hopperMover;
    Servo ringFlicker;

    boolean buttonIsHeld  = false;
    boolean otherButtonIsHeld = false;
    boolean positionUp = true;

    int timeToMoveIn = 500;
    double OUT_POSITION = .9;
    double IN_POSITION = 0;

    double initTime;




    public Hopper(robotBase BASE) {
        super(BASE);
        initServo();
    }
    public enum Position { COLLECT_POSITION, TRANSFER_POSITION, INIT_POSITION };
    public enum flickerPosition{IN_POSITION, OUT_POSITION}
    public void setHopperPosition ( Position targetPositon) {
        switch (targetPositon){
            case COLLECT_POSITION:
                hopperMover.setPosition(.65);
                break;

            case INIT_POSITION:
            case TRANSFER_POSITION:
                hopperMover.setPosition(.825);
                break;
        }

    }
    public void setFlickerPosition ( flickerPosition targetPositon) {
        switch (targetPositon){
            case IN_POSITION:
                ringFlicker.setPosition(IN_POSITION);
                break;

            case OUT_POSITION:
                ringFlicker.setPosition(OUT_POSITION);
                break;
        }

    }

    void initServo() {
        hopperMover = base.getMapper().mapServo("hopper");
        ringFlicker = base.getMapper().mapServo("ringFlicker");
    }

    public void moveHopperInTeleop(boolean button){

        if(button && !buttonIsHeld){
            buttonIsHeld = true;
            if(!positionUp){
                setHopperPosition(Position.TRANSFER_POSITION);
            }
            else {
                setHopperPosition(Position.COLLECT_POSITION);
            }
            positionUp = !positionUp;
        }
        if(!button){
            buttonIsHeld = false;
        }
    }
    public boolean moveFlicker(boolean button, boolean hasHitFirst, ElapsedTime time) {
        if((button && !otherButtonIsHeld) || !hasHitFirst) {
            if(otherButtonIsHeld == false) {
                initTime = time.milliseconds();
                otherButtonIsHeld = true;
            }
            setFlickerPosition(flickerPosition.IN_POSITION);
            if(time.milliseconds()-initTime < 600)
                return false;
            setFlickerPosition(flickerPosition.OUT_POSITION);
            if(!button){
                otherButtonIsHeld = false;
            }
                return true;
        }else{
            return true;
        }

    }
    @Override
    public void stop() {
        hopperMover.setPosition(hopperMover.getPosition());
    }
}
