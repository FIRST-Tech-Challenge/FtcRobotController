package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name = "Centerstage Auto Basic", group = "Concept")
public class Centerstage_AutoBasic extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.driveTrain.moveForward(5, 0.5);
        robot.driveTrain.Wait(0.25);
        robot.driveTrain.strafe(-36, 0.5);
    }
}
