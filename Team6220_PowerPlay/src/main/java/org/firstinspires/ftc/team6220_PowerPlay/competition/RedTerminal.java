package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;

@Autonomous(name = "Red Terminal", group = "Competition")
public class RedTerminal extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driveTurntable(0);
        int signal = detectAprilTag();

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(-0.25, 0.0, 0.0);
        sleep(1400);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.25, 0.0, 0.0);
        sleep(1400);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(500);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, -0.25, 0.0);
        sleep(1700);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(500);

        switch (signal) {
            case 0:
                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(-0.25, 0.0, 0.0);
                sleep(1500);

                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, 0.0, 0.0);
                break;

            case 1:
                break;

            case 2:
                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.25, 0.0, 0.0);
                sleep(1500);

                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, 0.0, 0.0);
                break;
        }
    }
}
