package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "prototype")
public class prototype extends OpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.sisteme(gamepad1);
        robot.movement(gamepad1);
    }

}
