package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedAutoShort extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);

        robot.setUpDrivetrainMotors();
        robot.autoForward(660.4);
        robot.autoForward(-609.6);
        robot.setHeading(90, 1);
        robot.autoForward(1219.2);

    }
}
