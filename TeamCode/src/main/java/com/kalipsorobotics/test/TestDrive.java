//package com.kalipsorobotics.test;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.kalipsorobotics.actions.drivetrain.DriveAction;
//import com.kalipsorobotics.localization.SparkfunOdometry;
//import com.kalipsorobotics.localization.WheelOdometry;
//import com.kalipsorobotics.modules.DriveTrain;
//import com.kalipsorobotics.modules.IMUModule;
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@Config
//class RobotConfig {
//    public static String robotName = "jimmeh";
//    public static String color = "red";
//}
//@TeleOp
//public class TestDrive extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
//        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
//        IMUModule imuModule = new IMUModule(opModeUtilities);
//        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
//        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, Math.toRadians(0));
//        DriveAction driveAction = new DriveAction(driveTrain);
//
//
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//
//        waitForStart();
//        while (opModeIsActive()) {
//
//            sparkfunOdometry.updatePosition();
//            wheelOdometry.updatePosition();
//
//
//            Log.d("purepursaction_debug_odo_sparkfun", sparkfunOdometry.getCurrentPosition().toString());
//
//            if (true) {
//                Log.d("purepursaction_debug_odo_wheel global", wheelOdometry.getCurrentPosition().toString());
//                Log.d("purepursaction_debug_odo_wheel counts", String.format("r=%.2f, l=%.2f b=%.2f t=%.4f, imu=%.4f",
//                        wheelOdometry.countRight(),
//                        wheelOdometry.countLeft(),
//                        wheelOdometry.countBack(),
//                        wheelOdometry.getCurrentPosition().getTheta(),
//                        wheelOdometry.getCurrentImuHeading()));
//            }
//
//            telemetry.addData("position", wheelOdometry.getCurrentPosition());
//            telemetry.addData("theta", wheelOdometry.getCurrentImuHeading());
//            telemetry.addLine("Hello, " + RobotConfig.robotName);
//
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay()
//                    .setAlpha(0.25)
//                    .setFill(RobotConfig.color)
//                    .fillRect(
//                            wheelOdometry.getCurrentPosition().getX(),
//                            wheelOdometry.getCurrentPosition().getY(),
//                            14, 14);
//            dashboard.sendTelemetryPacket(packet);
//            driveAction.move(gamepad1);
//            telemetry.update();
//        }
//    }
//}