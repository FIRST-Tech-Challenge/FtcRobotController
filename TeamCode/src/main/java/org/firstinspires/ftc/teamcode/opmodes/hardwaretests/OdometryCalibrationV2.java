package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

// Note: this class was inspired by the odometry calibration program made by Wizard.exe, FTC Team 9794
@Disabled
@Autonomous(name = "Manual Odometry Calibration", group = "Hardware Testers")
public class OdometryCalibrationV2 extends UpliftTele {

    UpliftRobot robot;
    DriveCommands drive;
    Odometry odom;
    double wheelBaseSeparation;
    double horizontalTickOffset;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        odom = new Odometry(robot);
    }

    @Override
    public void initAction() {

        // add necessary parameters for the REVhub imu
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        robot.imu.initialize(parameters);
        telemetry.clear();

        // Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status: ", "Init Complete");
        telemetry.update();
    }

    @Override
    public void bodyLoop() {
        // keep looping until angle equals, or exceeds 90 degrees
        while (getZAngle() < 90) {
            Utils.sleep(5);
        }

        // update angle value (in degrees)
        double angle = Math.toRadians(getZAngle());

        double encoderTotal = odom.getLeftTicks() - odom.getRightTicks();

        wheelBaseSeparation = (encoderTotal / angle) / UpliftRobot.COUNTS_PER_INCH;

        horizontalTickOffset = odom.getCenterTicks() / Math.toDegrees(angle);

        // Calibration complete
        telemetry.addData("Odometry System Calibration Status", "Calibration Complete");

        // Display calculated constants
        telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
        telemetry.addData("Horizontal Encoder Tick offset per degree", horizontalTickOffset);

        // Display raw values
        telemetry.addData("IMU Angle", getZAngle());
        telemetry.addData("Left Position", odom.getLeftTicks());
        telemetry.addData("Right Position", odom.getRightTicks());
        telemetry.addData("Center Position", odom.getCenterTicks());
        telemetry.update();

        robot.safeSleep(15000);
        stop();
    }

    @Override
    public void exit() {
        odom.stop();
    }

    private double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }
}
