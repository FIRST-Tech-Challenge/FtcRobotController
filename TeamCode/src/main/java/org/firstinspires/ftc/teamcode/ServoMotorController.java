package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoMotorController {
    
    private Servo servo;
    private boolean turnPos;

    public ServoMotorController(HardwareMap hm, String servoName) {
        servo = hm.get(Servo.class, servoName);
        turnPos = true;
    }

    public void runServoTurret(boolean turnDir) {
        servo.setPosition(newPos(turnDir));
        checkDirection();
    }

    private double newPos(boolean opposite){
        if (opposite){
            return servo.getPosition()-0.01;
        }
        return servo.getPosition()+0.01;
    }

    private void checkDirection(){
        if(servo.getPosition >= 0.3){
            Servo.setDirection(REVERSE);
        }
        if(servo.getPostion <= -0.3){
            Servo.setDirection(FORWARD);
        }
    }
}
