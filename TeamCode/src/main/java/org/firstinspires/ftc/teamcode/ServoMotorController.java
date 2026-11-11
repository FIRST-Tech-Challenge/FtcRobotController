package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoMotorController {
    
    // techicaly not its speed, it just is the added position that the servo attempts to reach
    private double turretSpeed;
    private Servo servo;

    public ServoMotorController(HardwareMap hm, String servoName) {
        servo = hm.get(Servo.class, servoName);
        servo.setPosition(0.5);
        turretSpeed = 0.001;
    }

    public void runServoTurret(boolean turnDir) {
        servo.setPosition(newPos(turnDir));
        checkDirection();
    }

    private double newPos(boolean opposite){
        if (opposite){
            return servo.getPosition() -turretSpeed;
        }
        return servo.getPosition() +turretSpeed;
    }

    private void checkDirection(){
        if(servo.getPosition() >= 1.0){
            servo.setDirection(Servo.Direction.REVERSE);
        }
        if(servo.getPosition() <= 0.0){
            servo.setDirection(Servo.Direction.FORWARD);
        }
    }
}