package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Command {
    public DcMotor Intake;
    public long time;
    public double speed;
    public long endTime;


    public Intake(HardwareMap hardwareMap, long Time, double Speed) {
        Intake = hardwareMap.dcMotor.get("Right_Intake");
        this.speed = Speed;
        this.time = Time;
    }

    public void start() {
        Intake.setPower(-speed);
        endTime = System.currentTimeMillis() + time;
    }
    public void execute() {

    }

    public void end() {
        Intake.setPower(0);
    }

    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime){
            return true;
        }
        else {
            return false;
        }
    }
}