package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.MainOpMode.driveAndLinslide;

public class dump {
    static Servo servo;
    static double timePressed; //records the amount of time that has passed.
    final static double enoughTime=0.0;
    static double prevTime=1;
    public enum states{READY, DUMPING, RECOVER}
    public static states state = states.READY;

    static double dumpStartTime=0;
    static double recoverStartTime=0;

    final static double endPos=0;
    final static double startPos=1;

    public static void setServo(Servo thisServo){
        servo = thisServo;
    }
    public static void startPos(){
        servo.setPosition(startPos);
    }

    public static void dumpFreight(ElapsedTime runtime){
        servo.setPosition(endPos);
        dumpStartTime = runtime.time();
    }

    public static void recover(ElapsedTime runtime){
        servo.setPosition(startPos);
        recoverStartTime=runtime.time();
    }

    public static void update(ElapsedTime runtime){
        if(runtime.time()-dumpStartTime>=1&&state.equals(states.DUMPING)){
            state=states.RECOVER;
            recover(runtime);
        }
        if(runtime.time()-recoverStartTime>=1&&state.equals(states.RECOVER)){
            state=states.READY;
        }
    }

    public static void re(Gamepad gamepad1, ElapsedTime runtime){

        if(gamepad1.y){
            timePressed+=runtime.time()-prevTime;  // purpose? could be causing problems
        }
        else{
            timePressed=0;
        }

        if(gamepad1.y&&timePressed>=enoughTime){  //problem is likely that we must push the dump button for a long time? (2nd and statement)
            state=states.DUMPING;
            dumpFreight(runtime);
        }

        prevTime=runtime.time();
        update(runtime);

    }

}


