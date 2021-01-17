package org.firstinspires.ftc.teamcode.action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class SetServoAction implements Action {

    String servoName;
    double position;
    public SetServoAction(String servoName, double position) {
        this.servoName = servoName;
        this.position = position;
    }


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        Servo servo = hardware.hardwareMap.get(Servo.class, servoName);
        if (servo != null) {
            servo.setPosition(position);
        }
        return true;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }

    @Override
    public Object getActionResult() {
        return null;
    }
}
