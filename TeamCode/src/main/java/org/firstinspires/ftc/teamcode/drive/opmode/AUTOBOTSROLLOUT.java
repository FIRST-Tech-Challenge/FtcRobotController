package org.firstinspires.ftc.teamcode.drive.opmode;

import android.view.animation.AnimationUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.List;

@Autonomous(name = "AUTOBOTSROLLOUT", group = "Concept")
public class AUTOBOTSROLLOUT extends LinearOpMode{

    //movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int targetPosition = 1;
    int currentPosition;

    //true == up
    boolean direction = true;
    double beginTime, currentTime, timeRemaining;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double distanceInches = 1;
        double placeHolderDistance = 1;

        Pose2d startingPose = new Pose2d(10, -60, Math.toRadians(0));
        drive.setPoseEstimate(startingPose);

        waitForStart();
        int targetPosition = 1000;
        while (opModeIsActive()) {
//            double command = control.update(targetPosition,
//                    motor.getCurrentPosition());
            TrajectorySequence AUTOBOTSROLLOUT = drive.trajectorySequenceBuilder(startingPose)
                    .splineTo(new Vector2d(40, -34), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                        telemetry.addData("Ran", "none");
                        drive.liftMotor1.setTargetPosition(1000);
                        drive.liftMotor2.setTargetPosition(1000);
                        drive.liftServo1.setPosition(1);
                        sleep(1000);
                        drive.doorServo.setPosition(0.7);
                    })
                    .waitSeconds(1)
                    .build();
            drive.followTrajectorySequence(AUTOBOTSROLLOUT);
            break;
        }
        waitForStart();
    }

//    public void liftUpdate(SampleMecanumDrive drive) {
//        currentPosition = drive.liftMotor1.getCurrentPosition();
//        if (targetPosition == 0){
//        }
//        else if (currentPosition < targetPosition && direction == true) {
//            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor2.setPower(power);
//        } else if (currentPosition > targetPosition && direction == false) {
//            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor2.setPower(power);
//        }
//        else if (currentPosition+10 > targetPosition && direction == true){
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor2.setPower(power);
//        }
//        else if (currentPosition+10 < targetPosition && direction == false){
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor1.setPower(power);
//        }
//    }
}
