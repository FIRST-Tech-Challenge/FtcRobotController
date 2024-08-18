package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewStuff.DetectPropPositionAction;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.GoToPropAction;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;
import org.firstinspires.ftc.teamcode.NewStuff.VisionPortalProcessor;

@Autonomous
public class TestAuto extends LinearOpMode {

    boolean isRedAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        Outtake outtake = new Outtake(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        DroneLauncher droneLauncher = new DroneLauncher(opModeUtilities);
        DriveTrain driveTrain;

        Log.d("vision", "opmode: making portal processor");
        VisionPortalProcessor visionPortalProcessor = new VisionPortalProcessor(opModeUtilities, isRedAlliance);
        Log.d("vision", "opmode: made portal processor");
        driveTrain = new DriveTrain(opModeUtilities);
//        robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));

        Log.d("vision", "opmode: making action");
         DetectPropPositionAction detectMarkerPos = new DetectPropPositionAction(visionPortalProcessor, false);
        //TurnDroneLauncherWheelAction droneLauncherWheelAction = new TurnDroneLauncherWheelAction(0, droneLauncher);
        Log.d("vision", "opmode: made action");

        waitForStart();

        GoToPropAction goToProp = new GoToPropAction(detectMarkerPos, detectMarkerPos.getPropLocation(), driveTrain, isRedAlliance);

        while (opModeIsActive()) {
            //Log.d("vision", "opmode: updating");
            detectMarkerPos.updateCheckDone();
            // Thread.sleep(10);
//            goToProp.updateCheckDone();

        }
    }
}
