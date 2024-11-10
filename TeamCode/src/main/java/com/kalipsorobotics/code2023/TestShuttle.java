//package com.kalipsorobotics.test;
//
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.kalipsorobotics.code2023.DroneLauncherX;
//import com.kalipsorobotics.code2023.OuttakeArmClamp;
//import com.kalipsorobotics.code2023.OuttakeArmPivot;
//import com.kalipsorobotics.code2023.OuttakeSlide;
//import com.kuriosityrobotics.shuttle.HardwareTaskScope;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import java.util.concurrent.TimeoutException;
//
//@Autonomous(name = "joshua")
//public class TestShuttle extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        try (var outer = HardwareTaskScope.open()){
//            try (var scope = HardwareTaskScope.open(TimeoutException.class)) {
//
//                OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
//                OuttakeSlide outtakeSlide = new OuttakeSlide(opModeUtilities);
//                OuttakeArmPivot outtakeArmPivot = new OuttakeArmPivot(opModeUtilities);
//                OuttakeArmClamp outtakeArmClamp = new OuttakeArmClamp(opModeUtilities);
//                DroneLauncherX droneLauncher = new DroneLauncherX(opModeUtilities);
//
//                //DriveTrain driveTrain = new DriveTrain(opModeUtilities);
//                //Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
//                //RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//
//                //outer.fork(odometry::run);
//
//                /*List<Point> pathPoints = new ArrayList<Point>() {{
//                    add(new Point(0, 0));
//                    add(new Point(600, 0));
//                    add(new Point(600, 600));
//                }};*/
//                outtakeArmPivot.goToZeroPos();
//                outtakeArmClamp.clamp();
//                droneLauncher.readyLauncher();
//                waitForStart();
//                //code
//                //scope.fork(() -> robotMovement.pathFollow(new Path(pathPoints)));
//                //scope.fork(() -> driveTrain.setTestPower(0.5));
//                scope.fork(() -> outtakeSlide.goToPosition(0.25));
//                scope.fork(droneLauncher::launch);
//                scope.fork(outtakeArmPivot::goToOuttakePos);
//
//                scope.join();
//
//                outtakeArmClamp.release();
//
//                Thread.sleep(1000);
//
//                outtakeArmPivot.goToZeroPos();
//                outtakeSlide.goToPosition(0);
//
//                scope.join();
//
//            } catch (TimeoutException e) {
//                throw new RuntimeException(e);
//            } finally {
//                outer.shutdown();
//                outer.join();
//            }
//        }
//    }
//}
