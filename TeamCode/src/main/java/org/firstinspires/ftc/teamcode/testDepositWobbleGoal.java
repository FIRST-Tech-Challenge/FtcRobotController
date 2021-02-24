package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name = "testDepositWobbleGoal")

public class testDepositWobbleGoal extends LinearOpMode {


    RobotClass robot = null;
    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();

        waitForStart();

        robot.depositWobbleGoal();

    }
}
