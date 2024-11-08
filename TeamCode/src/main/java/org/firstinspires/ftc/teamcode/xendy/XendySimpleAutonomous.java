package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.DriveChassis;

@Autonomous(name = "Forward Only Autonomous", group="XendySimpleTeleop")
public class XendySimpleAutonomous extends OpMode {
    DriveChassis chassis;
    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    public final double speed = 0.25;
    @Override
    public void loop() {
        chassis.leftBackMotor.setPower(speed);
        chassis.leftFrontMotor.setPower(speed);
        chassis.rightFrontMotor.setPower(speed);
        chassis.rightBackMotor.setPower(speed);
    }
}
