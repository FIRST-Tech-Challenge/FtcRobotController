package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driveGrabber(true);
        driveTurntable(1, Constants.TURNTABLE_DEFAULT_POSITION);

        waitForStart();

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.25, 0.0, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.25, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(-0.25, 0.0, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, -0.25, 0.0);
        sleep(1500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
    }
}
