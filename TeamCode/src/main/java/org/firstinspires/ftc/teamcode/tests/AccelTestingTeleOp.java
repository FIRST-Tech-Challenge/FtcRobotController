//package org.firstinspires.ftc.teamcode.tests;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.teamcode.powerplay.PowerPlayRobot;
//import org.firstinspires.ftc.teamcode.robotbase.IMUSubsystem;
//
//
//@TeleOp(name = "AccelTesting", group = "TeleOP")
//@Disabled
//public class AccelTestingTeleOp extends LinearOpMode {
//    private BNO055IMU imu;
//
//    private FtcDashboard dashboard;
//    private Telemetry dashboardTelemetry;
//
//    private Acceleration acceleration;
//
//    private double[] accel;
//    private double maxAccelX, maxAccelY, maxAccelZ;
//
//    @Override
//    public void runOpMode() {
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = this.dashboard.getTelemetry();
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            acceleration = imu.getLinearAcceleration();
//            accel = new double[]{acceleration.xAccel / 1000,
//                    acceleration.yAccel / 1000,
//                    acceleration.zAccel / 1000};
//
//            maxAccelX = accel[0] > maxAccelX ? accel[0] : maxAccelX;
//            maxAccelY = accel[1] > maxAccelY ? accel[1] : maxAccelY;
//            maxAccelZ = accel[2] > maxAccelZ ? accel[2] : maxAccelZ;
//
//            telemetry.addData("Accel X: ", accel[0]);
//            telemetry.addData("Accel Y: ", accel[1]);
//            telemetry.addData("Accel Z: ", accel[2]);
//            telemetry.addData("Max Accel X: ", maxAccelX);
//            telemetry.addData("Max Accel Y: ", maxAccelY);
//            telemetry.addData("Max Accel Z: ", maxAccelZ);
//
//            dashboardTelemetry.addData("Accel X: ", accel[0]);
//            dashboardTelemetry.addData("Accel Y: ", accel[1]);
//            dashboardTelemetry.addData("Accel Z: ", accel[2]);
//            dashboardTelemetry.addData("Max Accel X: ", maxAccelX);
//            dashboardTelemetry.addData("Max Accel Y: ", maxAccelY);
//            dashboardTelemetry.addData("Max Accel Z: ", maxAccelZ);
//
//            telemetry.update();
//            dashboardTelemetry.update();
//        }
//    }
//}