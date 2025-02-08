package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bots.AutomationBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.LimelightBot;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.bots.PedroPathingBot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
@Autonomous(name = "Pedro Auto", group = "Auto")
public class PedroAuto extends LinearOpMode {
    protected PedroPathingBot robot = new PedroPathingBot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.onLoop(10, "pedro auto");


            telemetry.update();
        }
    }

}
