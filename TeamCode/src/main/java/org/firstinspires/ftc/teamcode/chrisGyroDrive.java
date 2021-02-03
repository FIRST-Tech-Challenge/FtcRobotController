package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="chrisBot", name="PID Drive Test")

public class chrisGyroDrive extends LinearOpMode {
    chrisBot robot = new chrisBot();

    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false, false);

        waitForStart();

        //robot.halfGyroDrive(12, 0.3);
    }

}
