package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;

@Autonomous(name="Odometry Test", group="Tests")
@Disabled
public class OdometryTest extends LinearOpMode {

    protected OdometryBot robot = new OdometryBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.driveByGyroWithEncodersVertical(robot.DIRECTION_FORWARD, 40000, false, 1000, 5000);
        robot.sleep(30000);
    }
}
