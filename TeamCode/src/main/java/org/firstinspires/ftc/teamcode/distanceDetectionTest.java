package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "distanceDetectionTest")
public class distanceDetectionTest extends LinearOpMode {
    private DistanceSensor horizontalDistanceSensor, verticalDistanceSensor;
    private IMU imu;

    public void displaySensors() {
        double horizontalDistance = horizontalDistanceSensor.getDistance(DistanceUnit.CM);
        double verticalDistance = verticalDistanceSensor.getDistance(DistanceUnit.CM);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("horiz:", horizontalDistance);
        telemetry.addData("vert:", verticalDistance);
        telemetry.addData("orientation", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "horizontalDistanceSensor");
        verticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "verticalDistanceSensor");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        waitForStart();
        while (opModeIsActive()) {
            displaySensors();
        }
    }
}