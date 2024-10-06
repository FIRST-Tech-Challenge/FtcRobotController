package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {

    public DcMotor motor;
    private ElapsedTime runtime = new ElapsedTime();

    //Init hardware map
    HardwareMap hwMap = null;

    //Initializing hardware maps and motors for intake.
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motor = hwMap.get(DcMotor.class, "Intake");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void MoveIntake(double speed, boolean direction) {
        if (direction) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        if (!direction) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
        motor.setPower(speed);
    }
}