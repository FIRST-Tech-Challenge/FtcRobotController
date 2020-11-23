package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "Park_And_Shoot")
public class R_mid3_P2w_park extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        robot.moveServo(true);
        robot.moveWobbleGoalServo(false);
        robot.moveBackward(60, 0.5);
        robot.shootHighGoal(3);
        robot.moveBackward(20, 0.5);
        sleep(1500);
        robot.moveWobbleGoalServo(true);
        robot.moveBackward(-7, 0.5);
    }
}