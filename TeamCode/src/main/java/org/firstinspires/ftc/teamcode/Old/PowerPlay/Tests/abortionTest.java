package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//@Disabled

@Config
@Autonomous(name = "abortionTest")


public class abortionTest extends LinearOpMode {
    private SampleMecanumDrive roadrun;
    private PwPRobot robot = null;



    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new PwPRobot(this, true);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(41, 63.25, Math.toRadians(270));
        robot.setPoseEstimate(startPose);
        waitForStart();
        if (isStopRequested()) return;
        robot.setFirstLoop(true);
        while (opModeIsActive() && !isStopRequested() && getRuntime()<29.8&&(!robot.queuer.isFullfilled()||robot.queuer.isFirstLoop())) {
            logger.loopcounter++;
            if(!mainSequence()){
                robot.roadrun.breakFollowing();
                robot.roadrun.setMotorPowers(0,0,0,0);
                break;
            }
            robot.setFirstLoop(false);
            robot.roadrun.update();
            robot.updateClawStates();
        }
        robot.queuer.reset();
        while (opModeIsActive() && !isStopRequested() && getRuntime()<29.8&&(!robot.queuer.isFullfilled()||robot.queuer.isFirstLoop())) {
            logger.loopcounter++;
            parkSequence();
            robot.setFirstLoop(false);
            robot.roadrun.update();
            robot.updateClawStates();
        }
        robot.stop();
        if (getRuntime() > 29.8) {
            stop();
        }
    }
    private boolean mainSequence(){
        robot.splineTo(new Pose2d(41,40,Math.toRadians(270)), Math.toRadians(270),0);
        robot.closeClaw(false);
        robot.delay(1.5);
        if(robot.queuer.queue(true,true)){
            if(!robot.clawSwitch.isSwitched()){
                return false;
            }
        }
        robot.splineTo(new Pose2d(41, 63.25, Math.toRadians(270)),Math.toRadians(90),toRadians(180));
        robot.splineTo(new Pose2d(41,40,Math.toRadians(270)),Math.toRadians(270),0);
        return true;
    }
    private void parkSequence(){
        robot.splineTo(new Pose2d(20,40,Math.toRadians(180)),Math.toRadians(180),0);
    }
}
