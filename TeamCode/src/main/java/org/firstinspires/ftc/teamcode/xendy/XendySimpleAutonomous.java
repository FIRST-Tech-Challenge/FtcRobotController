package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Forward Only Autonomous", group="XendySimpleTeleop")
public class XendySimpleAutonomous extends OpMode {
    XendyChassis chassis;
    @Override
    public void init() {
        chassis = new XendyChassis(this);
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
