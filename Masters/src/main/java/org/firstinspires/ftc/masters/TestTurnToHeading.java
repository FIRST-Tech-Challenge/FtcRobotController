package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp
public class TestTurnToHeading extends LinearOpMode {
    RobotClass robot;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.turnToHeadingSloppy(.5,90);
        robot.pauseButInSecondsForThePlebeians(10);
    }
}
