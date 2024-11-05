
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name="Autonomous", group = "Real")
public class Parking_with_roadrunner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        robot.configureAutoSetting();

        int InchToTile= org.firstinspires.ftc.teamcode.Constants.INCH_TO_TILE;
        //tell the bot where it starts
        Pose2d initialPose = new Pose2d(1.5*InchToTile, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //decide where we should park
        boolean isTouchingLowRung=false;


        //in vector 2d, x-axis is the our-net-zone to opponent-ob-zone side,y-axis is our-net-zone to our-ob-zone
        //The robot starts from the 1 and a half tile from net zone,touching the wall facing opponent

        TrajectoryActionBuilder touchLowRung = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(2.5*InchToTile,1.5*InchToTile))
                                .strafeTo(new Vector2d(2.5*InchToTile,2.5*InchToTile));//trying to touch the low rung

        TrajectoryActionBuilder parkObzone = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0*InchToTile,6.5*InchToTile));//drive straight into ob zone


        waitForStart();

        Action trajectoryActionChosen;
        //decide where we should park
        if (isTouchingLowRung) {
            trajectoryActionChosen = touchLowRung.build();
        } else {
            trajectoryActionChosen = parkObzone.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                ));

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();



            drive.updatePoseEstimate();//show where the bot think itself at

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }

}
