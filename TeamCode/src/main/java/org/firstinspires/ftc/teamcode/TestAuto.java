package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewStuff.DetectPropPositionAction;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.FieldPosition;
import org.firstinspires.ftc.teamcode.NewStuff.GoToPropAction;
import org.firstinspires.ftc.teamcode.NewStuff.IMUModule;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;
import org.firstinspires.ftc.teamcode.NewStuff.VisionPortalProcessor;

import java.lang.reflect.Field;

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
        IMUModule imuModule = new IMUModule(opModeUtilities);
        FieldPosition fieldPosition = new FieldPosition();

        Log.d("vision", "opmode: making portal processor");
        VisionPortalProcessor visionPortalProcessor = new VisionPortalProcessor(opModeUtilities, isRedAlliance);
        Log.d("vision", "opmode: made portal processor");
        driveTrain = new DriveTrain(opModeUtilities);
//        robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));

        Log.d("vision", "opmode: making action");
        DetectPropPositionAction detectMarkerPos = new DetectPropPositionAction(visionPortalProcessor, fieldPosition, false);
        //TurnDroneLauncherWheelAction droneLauncherWheelAction = new TurnDroneLauncherWheelAction(0, droneLauncher);
        Log.d("vision", "opmode: made action");

        waitForStart();

        Log.d("auto", "heading is" + imuModule.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        Log.d("auto", "current lcoation is " + driveTrain.getfLeftTicks());

        GoToPropAction goToProp = new GoToPropAction(detectMarkerPos, fieldPosition, driveTrain, imuModule, visionPortalProcessor, isRedAlliance);

        while (opModeIsActive()) {
            //Log.d("vision", "opmode: updating");
            detectMarkerPos.updateCheckDone();
            // Thread.sleep(10);
            goToProp.updateCheckDone();

        }
    }
}
