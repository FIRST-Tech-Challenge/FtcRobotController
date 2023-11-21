package org.firstinspires.ftc.teamcode.McDonald;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

@Config
@Autonomous(name = "Auto Test", group = "Auto")
public class AutoTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        waitForStart();

        while(opModeIsActive()) {
            robot.moveRobot(.5, 10, 5);
            break;
        }
    }
}
