package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.GyroBot;

@Autonomous(name="Gyro Test", group="Tests")
@Disabled
public class GyroTest extends LinearOpMode {

    protected GyroBot robot = new GyroBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
//        robot.goToAnglePID(90);
//        robot.goToAngle(90, 0.18);
//        robot.sleep(2000);
//        robot.goToAnglePID(45);
//        robot.goToAngle(45, 0.18);
//        robot.sleep(2000);
//        robot.goToAnglePID(-90);
//        robot.goToAngle(-90, 0.18);
        robot.sleep(30000);
    }
}
