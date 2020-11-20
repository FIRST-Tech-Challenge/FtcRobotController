package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "ParkOdometry")
public class ParkOdometry extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        robot.moveAngleOdometry(0,60, 0.5);
        sleep(5000);
    }
}
