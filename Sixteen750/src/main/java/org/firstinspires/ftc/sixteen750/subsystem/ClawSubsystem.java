package org.firstinspires.ftc.sixteen750.subsystem;

import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.hardware.servo.Servo;

public class ClawSubsystem implements Subsystem {
    public static double OPEN_SERVO_POSITION = .8;
    public static double CLOSE_SERVO_POSITION = .5;
    public static double CARRY_SERVO_POSITION = .4;
    public static double RELEASE_SERVO_POSITION = .0;
    private Servo clawServo;
    private Servo flipperServo;

    public ClawSubsystem(Servo claw, Servo flipper){
        clawServo = claw;
        flipperServo = flipper;
    }
    public void open(){
        clawServo.setPosition(OPEN_SERVO_POSITION);
    }
    public void close(){
        clawServo.setPosition(CLOSE_SERVO_POSITION);
    }
    public void carry(){
        close();
        flipperServo.setPosition(CARRY_SERVO_POSITION);
    }
    public void release(){
        flipperServo.setPosition(RELEASE_SERVO_POSITION);
        open();
    }
}