package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="testForYaw")
public class testForYaw extends LinearOpMode {

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        while (opModeInInit()){


            IMU.Parameters myIMUparameters;

            myIMUparameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(myIMUparameters);
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Create Orientation variable
                Orientation myRobotOrientation;

                // Get Robot Orientation
                myRobotOrientation = imu.getRobotOrientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES
                );
                telemetry.addData("Values: ", myRobotOrientation);
            }
        }

    }

}
