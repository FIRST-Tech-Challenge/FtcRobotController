package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.Line;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFUltrasonic;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Ultrasonics Test")
public class RFUltrasonicTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false);
        BasicRobot robot = new BasicRobot(this, false);
        RFUltrasonic ultra = new RFUltrasonic("newUltra");
//        robot.roadrun.setPoseEstimate(new Pose2d(-38.5, -56, Math.toRadians(-90)));

        while (!isStarted()) {
            ultra.setLine(new Line(1,0,56, new Vector2d(56,-54), new Vector2d(56,-18)));
            robot.update();
        }
        while (!isStopRequested() && opModeIsActive() && !robot.queuer.isFullfilled()) {
//            ultra.check();
//            op.telemetry.addData("moving closer: ", ultra.getMovingCloser());
            op.telemetry.update();
//            packet.put("dist", ultra.getLinearRegressionDist());
            packet.put("rawDist", ultra.getDist());
//            op.telemetry.addData("check", ultra.detected);
            op.telemetry.addData("voltage", ultra.getVoltage());
            op.telemetry.addData("dist", ultra.getDist());
            robot.queuer.setFirstLoop(false);
            robot.update();
        }
//        robot.stop();
        stop();
    }
}
