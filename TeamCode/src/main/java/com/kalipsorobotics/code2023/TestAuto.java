/*
package com.kalipsorobotics.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.code2023.AlignRobotToAprilTagXAction;
import com.kalipsorobotics.code2023.AlignRobotToAprilTagYAction;
import com.kalipsorobotics.code2023.DetectPropPositionAction;
import com.kalipsorobotics.modules.DriveTrain;
//import org.firstinspires.ftc.teamcode.NewStuff.actions.code2023.DropPixelAction;
import com.kalipsorobotics.code2023.FieldPosition;
import com.kalipsorobotics.code2023.GoToBoardAction;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.code2023.Intake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.code2023.Outtake2023;
import com.kalipsorobotics.code2023.PropDetector;
import com.kalipsorobotics.code2023.VisionPortalManager;
import com.kalipsorobotics.code2023.DroneLauncher;

@Autonomous
public class TestAuto extends LinearOpMode {

    Boolean isRedAlliance = true;
    Boolean longPath = false;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        Outtake2023 outtake = new Outtake2023(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        DroneLauncher droneLauncher = new DroneLauncher(opModeUtilities);
        DriveTrain driveTrain;
        IMUModule imuModule = new IMUModule(opModeUtilities);
        FieldPosition fieldPosition = new FieldPosition(isRedAlliance, longPath);

        Log.d("vision", "opmode: making portal processor");
        VisionPortalManager visionPortalManager = new VisionPortalManager(opModeUtilities, isRedAlliance);
        Log.d("vision", "opmode: made portal processor");
        driveTrain = new DriveTrain(opModeUtilities);
//        robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));

        fieldPosition.setWantedAprTagId(PropDetector.PROP_POSITION.RIGHT, PropDetector.ALLIANCE_COLOR.RED);
        Log.d("auto", "wanted april tag id is " + fieldPosition.getWantedAprTagId());

        Log.d("vision", "opmode: making action");

        visionPortalManager.getVisionPortal().setProcessorEnabled(visionPortalManager.getAprilTagProcessor(), false);
        visionPortalManager.getVisionPortal().setProcessorEnabled(visionPortalManager.getPropProcessor(), false);


        ActionSet outer = new ActionSet();

        outer.scheduleSequential(new DetectPropPositionAction(visionPortalManager, fieldPosition, false));
        outer.scheduleSequential(new GoToBoardAction(fieldPosition, driveTrain, imuModule, visionPortalManager, isRedAlliance));
        // first detect marker
        outer.scheduleSequential(new AlignRobotToAprilTagXAction(fieldPosition,driveTrain, visionPortalManager));
        // then go to prop
        outer.scheduleSequential(new AlignRobotToAprilTagYAction(fieldPosition,driveTrain,visionPortalManager));
        //outer.scheduleSequential(new DropPixelAction(outtake));

        //todo combine both into a single action


        //TurnDroneLauncherWheelAction droneLauncherWheelAction = new TurnDroneLauncherWheelAction(0, droneLauncher);
        // Log.d("vision", "opmode: made action");


        // Log.d("auto", "heading is" + imuModule.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // Log.d("auto", "current lcoation is " + driveTrain.getfLeftTicks());

        // GoToPropAction goToProp = new GoToPropAction(detectMarkerPos, fieldPosition, driveTrain, imuModule, visionPortalProcessor, isRedAlliance);

        waitForStart();

        while (opModeIsActive() */
/* && !outer.getIsDone() *//*
) {
            outer.updateCheckDone();
        }
    }
}
*/
