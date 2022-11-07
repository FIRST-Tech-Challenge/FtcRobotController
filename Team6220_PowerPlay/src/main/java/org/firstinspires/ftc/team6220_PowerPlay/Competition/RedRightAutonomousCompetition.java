package org.firstinspires.ftc.team6220_PowerPlay.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "RedRight", group = "Competition")
public class RedRightAutonomousCompetition extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        servoGrabber.setPosition(0.45);
        driveTurntable(1, Constants.TURNTABLE_DEFAULT_POSITION);
        int signal = detectAprilTag();

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.25, 0.0, 0.0);
        sleep(1688);

        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveWithIMU(0.0, 0.0, 0.0);
        sleep(500);

        switch (signal) {
            case 0:
                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, -0.25, 0.0);
                sleep(1500);

                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, 0.0, 0.0);
                break;

            case 1:
                break;

            case 2:
                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, 0.25, 0.0);
                sleep(1550);

                IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveWithIMU(0.0, 0.0, 0.0);
                break;
        }
    }
}
