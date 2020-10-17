package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="FieldTrackingTest", group ="Auto")
public class FieldTrackingTester extends LinearOpMode {

    static FieldTracker tracker;
    BNO055IMU imu;

    public void setupIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public void runOpMode() {
        //CHANGE PARAMETERS FOR WEBCAM/GRAPHICS USAGE
        tracker = new FieldTracker(hardwareMap, telemetry, true, true);
        setupIMU();

        double imuAngle, trackerAngle;

        waitForStart();

        long previousTime = System.currentTimeMillis(), currentTime;

        while (opModeIsActive()) {

            imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            TargetInfo trackerInfo = tracker.getTargetInfo();

            telemetry.addData("IMU Rotation", imuAngle);

            if (trackerInfo != null) {
                trackerAngle = trackerInfo.zRotation;
                telemetry.addData("Tracker Z Rotation", trackerAngle);
                telemetry.addData("Difference", imuAngle - trackerAngle);
            }

            telemetry.addData("\n\nTracker Info", tracker.getTargetInfo());

            currentTime = System.currentTimeMillis();
            telemetry.addData("Update speed", currentTime - previousTime + " milliseconds");
            previousTime = currentTime;

            telemetry.update();
        }
    }

}
