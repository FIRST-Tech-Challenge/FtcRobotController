package org.firstinspires.ftc.teamcode.kaitlyn;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivetrainPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class kaitlynTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        waitForStart();
        robot.setUpDrivetrainMotors();
        int inchesToMove = 24;

        robot.fLeftMecanumController.state.integral = 0;
        robot.fLeftMecanumController.state.lastError = 0;
        robot.fLeftMecanumController.state.lastTime = 0;

        robot.straightController.state.integral = 0;
        robot.straightController.state.lastError = 0;
        robot.straightController.state.lastTime = 0;

        while (opModeIsActive()) {
            Log.d("parallel", "fLeft current position " + robot.getfLeft().getCurrentPosition());
            Log.d("parallel", "3 inch to tick" + robot.bRightMecanumController.convertInchesToTicks(inchesToMove));
//            telemetry.addData("fleft", robot.fLeft.getCurrentPosition());
//            telemetry.addData("3 inch to tick", robot.bRightMecanumController.convertInchesToTicks(3));
//            telemetry.update();
            //expected behavior: robot move 3 inches to the right
            //DrivetrainPowers drivetrainPowers = robot.straightParallelPowerPID(robot.fLeft.getCurrentPosition(), robot.bRightMecanumController.convertInchesToTicks(inchesToMove), 1);
            robot.drivetrain.setMotorPower(robot.getfLeft(),
                    robot.motorParallelPowerPWait(robot.getfLeft().getCurrentPosition(),
                    robot.bRightMecanumController.convertInchesToTicks(inchesToMove), 1, SystemClock.currentThreadTimeMillis(), 3000));
            //Log.d("parallel", "fleft power" + drivetrainPowers.fLeftPower);
            //robot.mecanumBlocking2(-24);
            //break;
        }
    }
}
