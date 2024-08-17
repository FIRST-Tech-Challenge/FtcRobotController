package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kalipsorobotics.fresh.DriveTrain;
import com.kalipsorobotics.fresh.localization.Odometry;
import com.kalipsorobotics.fresh.localization.RobotMovement;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewStuff.DetectPropPositionAction;
import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.GoToPropAction;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;

@Autonomous
public class TestAuto extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    DroneLauncher droneLauncher;
    VisionProcessor visionProcessor;
    RobotMovement robotMovement;
    DriveTrain driveTrain;
    Odometry odometry;
    OpModeUtilities opModeUtilities;

    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        outtake = new Outtake(opModeUtilities);
        intake = new Intake(opModeUtilities);
        droneLauncher = new DroneLauncher(opModeUtilities);

        Log.d("vision", "making processor");
        visionProcessor = new VisionProcessor(opModeUtilities, false);
        Log.d("vision", "made processor");
//        driveTrain = new DriveTrain(opModeUtilities);
//        robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));

        Log.d("vision", "making action");
        DetectPropPositionAction detectMarkerPos = new DetectPropPositionAction(visionProcessor, false);
        Log.d("vision", "made action");

        waitForStart();

//        GoToPropAction goToProp = new GoToPropAction(detectMarkerPos, detectMarkerPos.markerLocation);

        while (opModeIsActive()) {
            detectMarkerPos.updateCheckDone();
//            goToProp.updateCheckDone();

        }
    }
}
