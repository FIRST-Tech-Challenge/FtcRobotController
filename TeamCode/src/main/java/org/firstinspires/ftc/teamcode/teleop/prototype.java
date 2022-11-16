package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class prototype extends OpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.fourbar1.setPower(gamepad2.left_stick_y);
        robot.movement(gamepad1);
    }
}
