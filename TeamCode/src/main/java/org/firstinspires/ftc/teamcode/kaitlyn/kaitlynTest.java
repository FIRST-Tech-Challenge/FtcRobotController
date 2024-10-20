package org.firstinspires.ftc.teamcode.kaitlyn;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class kaitlynTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        waitForStart();
        robot.setUpDrivetrainMotors();
        while (opModeIsActive()) {
            Log.d("parallel", "fLeft current position " + robot.fLeft.getCurrentPosition());
            Log.d("parallel", "3 inch to tick" + robot.bRightMecanumController.convertInchesToTicks(3));
//            telemetry.addData("fleft", robot.fLeft.getCurrentPosition());
//            telemetry.addData("3 inch to tick", robot.bRightMecanumController.convertInchesToTicks(3));
//            telemetry.update();
             //expected behavior: robot move 3 inches to the right
            robot.setMotorPower(robot.mecanumParallelPowerPID(robot.fLeft.getCurrentPosition(), robot.bRightMecanumController.convertInchesToTicks(3)));
        }
    }
}
