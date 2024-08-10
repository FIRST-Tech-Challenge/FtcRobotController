package com.kalipsorobotics.fresh;

import com.kalipsorobotics.fresh.mechanisms.DroneLauncher;
import com.kalipsorobotics.fresh.mechanisms.OuttakeArmClamp;
import com.kalipsorobotics.fresh.mechanisms.OuttakeArmPivot;
import com.kalipsorobotics.fresh.mechanisms.OuttakeSlide;
import com.kuriosityrobotics.shuttle.HardwareTaskScope;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.fresh.localization.Odometry;
import com.kalipsorobotics.fresh.localization.RobotMovement;
import com.kalipsorobotics.fresh.math.Path;
import com.kalipsorobotics.fresh.math.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeoutException;

@Autonomous(name = "joshua")
public class TestShuttle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (var outer = HardwareTaskScope.open()){
            try (var scope = HardwareTaskScope.open(TimeoutException.class)) {

                OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
                OuttakeSlide outtakeSlide = new OuttakeSlide(opModeUtilities);
                OuttakeArmPivot outtakeArmPivot = new OuttakeArmPivot(opModeUtilities);
                OuttakeArmClamp outtakeArmClamp = new OuttakeArmClamp(opModeUtilities);
                DroneLauncher droneLauncher = new DroneLauncher(opModeUtilities);

                //DriveTrain driveTrain = new DriveTrain(opModeUtilities);
                //Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
                //RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);

                //outer.fork(odometry::run);

                /*List<Point> pathPoints = new ArrayList<Point>() {{
                    add(new Point(0, 0));
                    add(new Point(600, 0));
                    add(new Point(600, 600));
                }};*/
                outtakeArmPivot.goToZeroPos();
                outtakeArmClamp.clamp();
                droneLauncher.readyLauncher();
                waitForStart();
                //code
                //scope.fork(() -> robotMovement.pathFollow(new Path(pathPoints)));
                //scope.fork(() -> driveTrain.setTestPower(0.5));
                scope.fork(() -> outtakeSlide.goToPosition(0.25));
                scope.fork(droneLauncher::launch);
                scope.fork(outtakeArmPivot::goToOuttakePos);

                scope.join();

                outtakeArmClamp.release();

                Thread.sleep(1000);

                outtakeArmPivot.goToZeroPos();
                outtakeSlide.goToPosition(0);

                scope.join();

            } catch (TimeoutException e) {
                throw new RuntimeException(e);
            } finally {
                outer.shutdown();
                outer.join();
            }
        }
    }
}
