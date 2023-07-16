package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Mechanism.ColorSensorMech;
import org.firstinspires.ftc.teamcode.Mechanism.LinearSlides;
import org.firstinspires.ftc.teamcode.Mechanism.RetractOdo;

@Autonomous
public class ColorSensorTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private LinearSlides linearSLides;
    private RetractOdo retractOdo;
    private ColorSensorMech colorSensorMech;
    public boolean coneTransportedSetup = false;
    private double startX = 36.0;
    private double startY = 65.0;
    public double startHeading;
    private double strafeVal;
    private int colorSensorDetection;

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        linearSLides = new LinearSlides(telemetry, hardwareMap);
        colorSensorMech = new ColorSensorMech(telemetry, hardwareMap);
        retractOdo = new RetractOdo(telemetry, hardwareMap);

        telemetry.addData("At least something is happening", "Hello");

        while (!isStarted() && !isStopRequested()) {

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            if (!coneTransportedSetup) {
                retractOdo.unretractOdometryServos();
                linearSLides.linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSLides.setGripperPosition(.75);
                sleep(2000);
                sleep(2000);
                coneTransportedSetup = true;
                telemetry.addData("Has gone into coneTransporter", "Hello");
                telemetry.update();
            }
        }

        startHeading = Math.toRadians(270);
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);

        strafeVal = colorSensorMech.isDetectingBlueLine();
        telemetry.addData("ColorSensorStatus", colorSensorDetection);
        if(colorSensorDetection == 1){
            telemetry.addData("FAIL", "The value is 1");
            telemetry.update();
            sleep(5000);
            strafeVal = 20;
        } else if (colorSensorDetection == 2) {
            telemetry.addData("FAIL", "The value is 2");
            telemetry.update();
            sleep(5000);
            strafeVal = -1;
        } else if (colorSensorDetection == 3) {
            telemetry.addData("FAIL", "The value is 3");
            telemetry.update();
            sleep(5000);
            strafeVal = 0.0001;
        } else {
            telemetry.addData("FAIL", "The value is -1");
            telemetry.update();
            sleep(5000);
            strafeVal = 0.0001;

        }

        TrajectorySequence ColorTest = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .strafeRight(strafeVal)
                .waitSeconds(3)
                .build();
        drive.followTrajectorySequenceAsync(ColorTest);

    }
}
