
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
        Pose2d initialPose = new Pose2d(1.5*InchToTile, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        boolean isTouchingLowRung=false;

        TrajectoryActionBuilder touchLowRung = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(1.5*InchToTile,2.5*InchToTile))
                                .strafeTo(new Vector2d(2.5*InchToTile,2.5*InchToTile));

        TrajectoryActionBuilder parkObzone = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(6.5*InchToTile,0*InchToTile));


        waitForStart();

        Action trajectoryActionChosen;
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



            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }

}
