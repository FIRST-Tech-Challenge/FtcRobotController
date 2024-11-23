package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber extends SubsystemBase {

    public ServoEx servo;
    private Telemetry tm;
    public boolean Active = false;
    private int CurrentAngle;
    private int MinimumAngle = -250;
    private int MaximumAngle = 250;
    private int Speed = 1; //default speed is 1


    public Climber(HardwareMap hardwareMap, Telemetry telemetry){
        tm = telemetry;
        //TODO: fix this name from config
        servo = new SimpleServo(hardwareMap, "climber", MinimumAngle, MaximumAngle);
        servo.setInverted(false);
        //setAngle(0);
    }

    public void addFifteen() {
        setAngle(0);
    }
    public void subFifteen() {
        setAngle(40);
    }

    public void SetSpeed(int speed) {
        Speed = speed;
    }
    public void Goto(int angle) {
        CurrentAngle = angle;
        setAngle(angle);
    }
    public void AddDegree() {
        if (Active && CurrentAngle < MaximumAngle) {
            CurrentAngle += Speed;
            setAngle(CurrentAngle);
        }
    }

    public void RemoveDegree() {
        if (Active && CurrentAngle > MinimumAngle) {
            CurrentAngle -= Speed;
            setAngle(CurrentAngle);
        }
    }

    public void setAngle(double angle){
        tm.addData("climber set angle", angle);
        if (this.Active == true) {
            servo.turnToAngle(angle);
        }
    }
    public void open(){
        setAngle(200);
    }
    public void close(){
        setAngle(-25);
    }

    @Override
    public void periodic(){
        tm.addData("climber active", this.Active);
        tm.addData("climber angle", servo.getAngle());
    }
}