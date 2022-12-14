package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.ConeDetection;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        int signal = detectAprilTag();

        driveInches(1, 0);
        driveInches(1, 180);

        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        driveSlides(300);

        driveInches(52, 0);
        driveInches(12, 270);

        driveSlides(Constants.SLIDE_HIGH);
        driveInches(2, 0);
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        driveInches(2, 180);
        driveSlides(800);

        driveInches(30, 90);

        detectGrab();

        while (Math.abs(coneDetectionPipeline.distance) > 50 && opModeIsActive()) {
            motorFL.setPower(-0.25);
            motorFR.setPower(0.25);
            motorBL.setPower(-0.25);
            motorBR.setPower(0.25);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        while (coneDetectionPipeline.coneSize > 3000 && opModeIsActive()) {
            motorFL.setPower(0.25);
            motorFR.setPower(0.25);
            motorBL.setPower(0.25);
            motorBR.setPower(0.25);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        driveSlides(stackHeight * 60);
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        driveSlides(800);

        driveInches(46, 180);

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
