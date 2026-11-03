package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoMotorController {
    
    private Servo servo;
    public ServoMotorController(HardwareMap hm, String servoName) {
        servo = hm.get(Servo.class, servoName);
    }

    public void runServo() {
        servo.setPosition(1);
    }
}
