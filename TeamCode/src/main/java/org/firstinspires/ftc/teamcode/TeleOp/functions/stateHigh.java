package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.MainOpMode.driveAndLinslide;

public class stateHigh {
    static Servo servo;
    static double timePressed; //records the amount of time that has passed.
    final static double enoughTime=1;
    static double prevTime;
    public enum states{READY, DUMPING, RECOVER};
    public static states state = states.READY;

    final static double endPos=0;
    final static double startPos=1;

    public static void setServo(Servo thisServo){
        servo = thisServo;
    }
    static void startPos(){
        servo.setPosition(startPos);
    }
    static void dump(){
        servo.setPosition(endPos);
    }

    static void recover(){
        servo.setPosition(startPos);
    }

    static void update(driveAndLinslide.states linSlideState){
        if(servo.getPosition()==endPos){
            state=states.RECOVER;
            recover();
        }
        if(servo.getPosition()==startPos&&linSlideState==driveAndLinslide.states.HIGH){
            state=states.READY;
        }
    }

    public static void re(Gamepad gamepad1, ElapsedTime runtime, driveAndLinslide.states linSlideState){
        if(gamepad1.y&&timePressed>=enoughTime&&state==states.READY){
            state=states.DUMPING;
            dump();
        }
        if(gamepad1.y){
            timePressed+=runtime.time()-prevTime;
        }
        else{
            timePressed=0;
        }

        if(state==states.DUMPING){
            dump();
        }

        if(state==states.RECOVER){
            recover();
        }
        update(linSlideState);

    }

}


