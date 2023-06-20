package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@TeleOp(name = "HardwareTestTrack")
public class HardwareTestTrack extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.cv.observeSleeve();
        Pose2d startPose =  new Pose2d(36, 63.25, Math.toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);//        robot.cv.observeStick();
//        robot.cv.observeCone();
        waitForStart();
        robot.cv.observeStick();
        resetRuntime();
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.update();
        while(!isStopRequested()){
            logger.loopcounter++;
            if(gamepad1.dpad_right){
                robot.roadrun.followTrajectory(robot.roadrun.trajectoryBuilder(robot.roadrun.getPoseEstimate()).strafeRight(47).build());
            }
            if(gamepad1.dpad_up){
                robot.roadrun.followTrajectory(robot.roadrun.trajectoryBuilder(robot.roadrun.getPoseEstimate()).forward(47).build());
            }
            double[] vals = {gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x};
            robot.roadrun.setWeightedDrivePower(new Pose2d(vals[1],vals[0],vals[2]));
            robot.field.lookingAtPole();
            telemetry.addData("polePose",robot.field.calcPolePose(robot.roadrun.getPoseEstimate()));
            telemetry.addData("dropPose",robot.field.polePos());
            robot.roadrun.update();
            telemetry.update();
        }
        robot.stop();
    }
}
