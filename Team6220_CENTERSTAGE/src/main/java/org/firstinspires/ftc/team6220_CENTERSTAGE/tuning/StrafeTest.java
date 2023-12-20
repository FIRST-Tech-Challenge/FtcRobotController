package org.firstinspires.ftc.team6220_CENTERSTAGE.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;
/*
This is part of roadrunner's calibration classes.
 */
@Disabled
@TeleOp
public final class StrafeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        final double DISTANCE = 60;

        double axialGain        = MecanumDrive.PARAMS.axialGain;
        double axialVelGain     = MecanumDrive.PARAMS.axialVelGain;
        double headingGain      = MecanumDrive.PARAMS.headingGain;
        double headingVelGain   = MecanumDrive.PARAMS.headingVelGain;
        double lateralGain      = MecanumDrive.PARAMS.lateralGain;
        double lateralVelGain   = MecanumDrive.PARAMS.lateralVelGain;

        MecanumDrive.PARAMS.axialGain = 0;
        MecanumDrive.PARAMS.axialVelGain = 0;
        MecanumDrive.PARAMS.headingGain = 0;
        MecanumDrive.PARAMS.headingVelGain = 0;
        MecanumDrive.PARAMS.lateralGain = 0;
        MecanumDrive.PARAMS.lateralVelGain = 0;
        
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        int initPos = drive.leftFront.getCurrentPosition();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE)) // Strafe left 60"
                        .build());

        int finalPos = drive.leftFront.getCurrentPosition();
        int ticksTraveled = Math.abs(finalPos - initPos);
        telemetry.addData("ticks traveled", ticksTraveled);
        telemetry.addData("lateralInPerTick", "Measured Distance / ticks traveled");
        telemetry.update();

        MecanumDrive.PARAMS.axialGain = axialGain;
        MecanumDrive.PARAMS.axialVelGain = axialVelGain;
        MecanumDrive.PARAMS.headingGain = headingGain;
        MecanumDrive.PARAMS.headingVelGain = headingVelGain;
        MecanumDrive.PARAMS.lateralGain = lateralGain;
        MecanumDrive.PARAMS.lateralVelGain = lateralVelGain;

        while (opModeIsActive())
            ;
    }
}
