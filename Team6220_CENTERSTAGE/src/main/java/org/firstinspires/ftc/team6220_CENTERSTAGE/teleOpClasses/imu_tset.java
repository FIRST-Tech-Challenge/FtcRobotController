package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

@Disabled
@TeleOp(name="imu_tset", group ="amogus")
public class imu_tset extends LinearOpMode {


    // holds heading from imu read which is done in roadrunner's mecanum drive class for us
    double currentHeading = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {

            // get heading from imu in degrees
            currentHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            telemetry.addData("imu reading", currentHeading);

            telemetry.update();
        }
    }
}