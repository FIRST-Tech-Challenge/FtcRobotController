package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CustomPID;

public class Claw {
    private Servo claw;
    private Servo rotate;


    public void init(HardwareMap hwMap){
        this.claw = hwMap.get(Servo.class, "claw");
        this.claw.setDirection(Servo.Direction.REVERSE);
        this.rotate = hwMap.get(Servo.class, "rotate");
        this.rotate.setDirection(Servo.Direction.REVERSE);
    }

    public void setClaw(double pos) {
        this.claw.setPosition(pos);
    }

    public void setRotate(double pos){
        this.rotate.setPosition(pos);
    }

    public double getPos(){return this.claw.getPosition();}
    public double getRotate(){return this.rotate.getPosition();}
}
