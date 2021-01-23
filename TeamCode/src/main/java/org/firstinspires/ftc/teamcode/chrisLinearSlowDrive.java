package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="chrisBot", name="Linear Slowing Drive Test")

public class chrisLinearSlowDrive extends LinearOpMode {
    chrisBot robot = new chrisBot();

    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false, false);

        waitForStart();

        robot.linearSlowEncoderDrive(240, 1);

    }

}
