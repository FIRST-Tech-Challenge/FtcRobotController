package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private HardwareMap hw;
    private ServoEx clawServo;

    public Claw(HardwareMap hw, String name){
        this.hw = hw;
        clawServo = new SimpleServo(hw,name, 0, 180);
    }

    public void open(){
        clawServo.setPosition(0);
    }

    public void close(){
        clawServo.setPosition(1);
    }



}
