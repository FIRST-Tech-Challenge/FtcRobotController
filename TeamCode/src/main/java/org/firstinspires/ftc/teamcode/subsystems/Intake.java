package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class Intake implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx intakeMotor;
    private double motorPosition = 0;

    public Intake(Robot robot) {
        intakeMotor = robot.getMotor("intakeMotor");

    }





    public void setPower(double power) {
        this.motorPosition = power;


        // set encode to new position
    }


    @Override
    public void update(TelemetryPacket packet) {intakeMotor.setPower(motorPosition);}
}
