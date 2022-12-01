package org.firstinspires.ftc.teamcode.opmodes.tests;

import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
//@Disabled
public class motortest extends LinearOpMode{
    @Override
    public void runOpMode() {
        TurtleRobotAuto robot = new TurtleRobotAuto(this);
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            robot.rightfrontmotor.setPower(1);
            sleep(1000); robot.rightfrontmotor.setPower(0);
            robot.leftfrontmotor.setPower(1); sleep(1000);
            robot.leftfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(1); sleep(1000);robot.rightbackmotor.setPower(0);
            robot.leftbackmotor.setPower(1);sleep(1000); robot.leftbackmotor.setPower(0);
            sleep(1000);
            robot.rightfrontmotor.setPower(0);
            robot.leftfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(0);
            robot.leftbackmotor.setPower(0);
        }
    }
}
