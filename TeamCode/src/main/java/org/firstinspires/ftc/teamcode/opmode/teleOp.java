package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp
@Autonomous
public class teleOp extends OpMode {
    RobotContainer robot =new RobotContainer(hardwareMap);
    @Override
    public void init() {
        robot.bindCommands();
    }

    @Override
    public void loop() {
        robot.run();
    }
}
