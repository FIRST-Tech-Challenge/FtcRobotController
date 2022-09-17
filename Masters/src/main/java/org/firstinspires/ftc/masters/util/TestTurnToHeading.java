package org.firstinspires.ftc.masters.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.RobotClass;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//@TeleOp(name="testTurn", group ="test")
@Disabled
public class TestTurnToHeading extends LinearOpMode {
    RobotClass robot;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.turnToHeadingSloppy(.5,45, 60);
        robot.pauseButInSecondsForThePlebeians(3);
        robot.turnToHeadingSloppy(.5,90,60);
        robot.pauseButInSecondsForThePlebeians(3);
    }
}
