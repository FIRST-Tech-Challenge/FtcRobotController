package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Disabled
@TeleOp(name="DistSensTele")
public class DistanceSensorTeleOp extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.cv.observeSleeve();
        Pose2d startPose = new Pose2d(41, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);//        robot.cv.observeStick();
        waitForStart();
        resetRuntime();
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.update();
        while(!isStopRequested()){
            logger.loopcounter++;
            robot.printLR();
            telemetry.update();
        }
        robot.stop();
    }
}
