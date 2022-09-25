package org.firstinspires.ftc.teamcode.opmodes.tests;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="hello world")
public class hello_world extends LinearOpMode{
    @Override
    public void runOpMode() {
        TurtleRobot robot = new TurtleRobot(this);
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            robot.rightfrontmotor.setPower(1);
            robot.leftfrontmotor.setPower(1);
            robot.rightbackmotor.setPower(1);
            robot.leftbackmotor.setPower(1);
            sleep(1000);
            robot.rightfrontmotor.setPower(0);
            robot.leftfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(0);
            robot.leftbackmotor.setPower(0);
        }
    }
}
