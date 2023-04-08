package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Disabled

@Config
@Autonomous(name = "SimpleTest")


public class SimpleTest extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    double[] stackPos = {440,320,173,53,0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(42, 63.25, Math.toRadians(270));
        robot.roadrun.setPoseEstimate(startPose);
        while(!isStarted()){
            telemetry.addData("pos",robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
        }
        resetRuntime();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested() && getRuntime()<28) {
            logger.loopcounter++;
            robot.liftToPosition(400);
            robot.waitForFinish();
            robot.liftToPosition(800);
            robot.waitForFinish();
            robot.liftToPosition(1200);
            robot.waitForFinish();
            robot.liftToPosition(1600);
            robot.waitForFinish();
            robot.liftToPosition(1200);
            robot.waitForFinish();
            robot.liftToPosition(800);
            robot.waitForFinish();
            robot.liftToPosition(400);
            robot.waitForFinish();
            robot.liftToPosition(0);
            robot.waitForFinish();
            robot.openClaw();
            robot.waitForFinish();
            robot.closeClaw();
            robot.waitForFinish();

            robot.setFirstLoop(false);
            //robot.liftToTargetAuto();
            robot.roadrun.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
        }
        robot.stop();
        if(getRuntime()>29.8){
            stop();
        }
    }
}
