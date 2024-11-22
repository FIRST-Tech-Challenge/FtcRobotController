
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
import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name="Drive-Base-Auto", group = "Real")
public class DrivebaseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        robot.configureAutoSetting();

        int allianceNumber = 1;//blue = 1, red = -1  //to change, not possible.


        //tell the bot where it starts
        Pose2d initialPose = new Pose2d(-3 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE*allianceNumber, Math.toRadians(90 * allianceNumber));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //decide where we should park
        boolean isGettingNeutral=true;
        boolean isTouchingLowRung=false;

        Pose2d updatedPose = new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble());


        TrajectoryActionBuilder getNeutral = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-1 * Constants.INCH_TO_TILE * allianceNumber,2 * Constants.INCH_TO_TILE * allianceNumber),Math.toRadians(270 * allianceNumber))
                .strafeTo(new Vector2d(-2.8 * Constants.INCH_TO_TILE * allianceNumber,2.8 * Constants.INCH_TO_TILE * allianceNumber));

        TrajectoryActionBuilder touchLowRung = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,1.5 * Constants.INCH_TO_TILE * allianceNumber))
                .strafeTo(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,0.5 * Constants.INCH_TO_TILE * allianceNumber));//trying to touch the low rung

        TrajectoryActionBuilder parkObzone = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-2.5 * Constants.INCH_TO_TILE * allianceNumber,-2.5 * Constants.INCH_TO_TILE * allianceNumber));//drive straight into ob zone




        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();



            drive.updatePoseEstimate();//show where the bot think itself at

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }

        Action chosenGettingBlockAction;

        if(isGettingNeutral){
            chosenGettingBlockAction = getNeutral.build();
        }else{
            chosenGettingBlockAction = parkObzone.build();
        };

        Action chosenParkingAction;
        //decide where we should park
        if (isTouchingLowRung) {
            chosenParkingAction = touchLowRung.build();
        } else {
            chosenParkingAction = parkObzone.build();
        }



        Actions.runBlocking(
                new SequentialAction(
                        chosenGettingBlockAction,
                        chosenParkingAction

                ));//execute the planned action


    }

}
