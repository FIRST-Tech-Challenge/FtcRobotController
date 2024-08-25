package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewStuff.ActionSet;
import org.firstinspires.ftc.teamcode.NewStuff.AlignRobotToAprilTagXAction;
import org.firstinspires.ftc.teamcode.NewStuff.AlignRobotToAprilTagYAction;
import org.firstinspires.ftc.teamcode.NewStuff.DetectPropPositionAction;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.FieldPosition;
import org.firstinspires.ftc.teamcode.NewStuff.GoToPropAction;
import org.firstinspires.ftc.teamcode.NewStuff.IMUModule;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.MoveRobotStraightInchesAction;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;
import org.firstinspires.ftc.teamcode.NewStuff.PropDetector;
import org.firstinspires.ftc.teamcode.NewStuff.TurnRobotAction;
import org.firstinspires.ftc.teamcode.NewStuff.VisionPortalProcessor;

import java.lang.reflect.Field;

@Autonomous
public class TestAuto extends LinearOpMode {

    Boolean isRedAlliance = true;
    Boolean longPath = false;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        Outtake outtake = new Outtake(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        DroneLauncher droneLauncher = new DroneLauncher(opModeUtilities);
        DriveTrain driveTrain;
        IMUModule imuModule = new IMUModule(opModeUtilities);
        FieldPosition fieldPosition = new FieldPosition(isRedAlliance, longPath);

        Log.d("vision", "opmode: making portal processor");
        VisionPortalProcessor visionPortalProcessor = new VisionPortalProcessor(opModeUtilities, isRedAlliance);
        Log.d("vision", "opmode: made portal processor");
        driveTrain = new DriveTrain(opModeUtilities);
//        robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));

        fieldPosition.setWantedAprTagId(PropDetector.PROP_POSITION.RIGHT, PropDetector.ALLIANCE_COLOR.RED);

        Log.d("vision", "opmode: making action");

        ActionSet outer = new ActionSet();

        // first detect marker
        outer.scheduleSequential(new AlignRobotToAprilTagXAction(fieldPosition,driveTrain,visionPortalProcessor));
        // then go to prop
        outer.scheduleSequential(new AlignRobotToAprilTagYAction(fieldPosition,driveTrain,visionPortalProcessor));

        //todo combine both into a single action

        // DetectPropPositionAction detectMarkerPos = new DetectPropPositionAction(visionPortalProcessor, fieldPosition, false);
        //TurnDroneLauncherWheelAction droneLauncherWheelAction = new TurnDroneLauncherWheelAction(0, droneLauncher);
        // Log.d("vision", "opmode: made action");

        // Log.d("auto", "heading is" + imuModule.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // Log.d("auto", "current lcoation is " + driveTrain.getfLeftTicks());

        // GoToPropAction goToProp = new GoToPropAction(detectMarkerPos, fieldPosition, driveTrain, imuModule, visionPortalProcessor, isRedAlliance);

        waitForStart();

        while (opModeIsActive()) {
            outer.updateCheckDone();
        }
    }
}
