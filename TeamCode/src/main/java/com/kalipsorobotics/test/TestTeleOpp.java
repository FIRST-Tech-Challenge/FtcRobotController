package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.localization.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;

@TeleOp
public class TestTeleOpp extends LinearOpMode {

    DriveTrain driveTrain;
    Odometry odometry;

    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = new DriveTrain(opModeUtilities);
        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        IMUModule imu = new IMUModule(opModeUtilities);


        //define the actions; enter a condition (if any), target ticks, and module
//        TurnIntakeWheelAction action1 = new TurnIntakeWheelAction(1000, intake);
//        MoveLSAction action2 = new MoveLSAction(action1,1000, outtake);
//        TurnIntakeWheelAction action3 = new TurnIntakeWheelAction(action2, 2000, intake);
//        MoveLSAction action4 = new MoveLSAction(action2, 2000, outtake);

//        //build a chain of actions
//        ActionSet outer = new ActionSet();
//
//        ActionSet actions1 = new ActionSet();
//        actions1.scheduleParallel(new MoveLSAction(500, outtake));
//        actions1.scheduleParallel(new TurnIntakeWheelAction(500, intake));
//
//        ActionSet actions2 = new ActionSet();
//        actions2.scheduleSequential(new TurnDroneLauncherWheelAction(500, droneLauncher));
//        actions2.scheduleSequential(new MoveTrayClampAction(0.5, outtake));
//        actions2.scheduleSequential(new WaitAction(5));
//        // outer.scheduleSequential(bundle);
//
//        MoveLSAction action3 = new MoveLSAction(actions2, 0, outtake);
//
//        outer.scheduleParallel(actions1);
//        outer.scheduleParallel(actions2);
//        outer.scheduleSequential(action3);

        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, odometry);
        purePursuitAction.addPoint(0,0);
        //purePursuitAction.addPoint(400,0);
        purePursuitAction.addPoint(0,-400);
        //purePursuitAction.addPoint(400,-400);

        waitForStart();

        while (opModeIsActive()) {

            //update actions, run when given condition is true
//            action1.update();
//            action2.update();
//            action3.update();
//            action4.update();

            odometry.updatePosition();
            Log.d("position", odometry.getCurrentPosition().toString());
            purePursuitAction.updateCheckDone();
            //driveTrain.setPower(0.2,-0.2,0.2,-0.2);

//            telemetry.addData("outtake ticks", outtake.lsFront.getCurrentPosition());
//            telemetry.addData("drone launcher ticks", droneLauncher.wheel.getCurrentPosition());
//            telemetry.update();

        }
    }
}
