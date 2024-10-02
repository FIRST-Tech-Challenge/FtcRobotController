package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SplineHeadingPath;
import com.acmerobotics.roadrunner.QuinticSpline1d;
import com.acmerobotics.roadrunner.QuinticSpline2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.CachingExposureControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Template Autoop", group = "16481-Example")
public class Guide extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-20, 60, Math.toRadians(270)));

        //^ double check the starting position, robot is using this as (0, 60) **Why is x = ?

        Action TrajectoryForwardToSample = drive.actionBuilder(drive.pose)

                //line up to score sample
                .splineTo(new Vector2d(5, 8), Math.toRadians(0))

                //down score sample
                .strafeTo(new Vector2d(5, 55))

                //up to line up scoring second sample
                .strafeTo(new Vector2d(5, 8))

                //back to line up scoring second sample
                .strafeTo(new Vector2d(-5,8))

                //down to score second sample
                .strafeTo(new Vector2d(10,55))

                //back to park in observation
                .strafeTo(new Vector2d(-100, 70))

                //strafe down into observation
                .strafeTo(new Vector2d(-105, 80))

                .build();

      //  Action TrajectoryScoring3Blue = drive.actionBuilder(drive.pose)

            //    .
             //   .build();



     /*
        Action TrajectoryParkingObservation = drive.actionBuilder(drive.pose)
                .lineToX(-50)
                .build();
*/

     /*
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToX(10)
                .build();

        Action TrajectoryAction2 = drive.actionBuilder(new Pose2d(15, 20, 0))
               .splineTo(new Vector2d(5, 5), Math.toRadians(90))
                .build();
        */


        while (!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryForwardToSample,
                        new Action() {
                        // This action and the following action do the same thing

                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Action!");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Only that this action uses a Lambda expression to reduce complexity
                        (telemetryPacket) -> {
                            telemetry.addLine("Action!");
                            telemetry.update();
                            return false; // Returning true causes the action to run again, returning false causes it to cease
                        }
                        )
        );
    }
}