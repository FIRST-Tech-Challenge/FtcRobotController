package org.firstinspires.ftc.teamcode.AutoControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

public class AutoControl extends OpMode {

    RobotClass robot;
    @Override
    public void init() {
        robot = new RobotClass(hardwareMap);
    }

    @Override
    public void loop() {

    }
    @Override
    public void stop(){

        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.drivetrain.driveMotors[0].setPower(0);
        robot.drivetrain.driveMotors[1].setPower(0);
        robot.drivetrain.driveMotors[2].setPower(0);
        robot.drivetrain.driveMotors[3].setPower(0);
    }
}
