package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Autonomous_root extends LinearOpMode {

    OpenCvCamera camera;
    SleeveDetector aprilTagDetectionPipeline;

    //sleeve
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    //c&p :(
    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 2-0.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    // Lens intrinsics
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    ////////
    private enum TeamColor {red, blue}
    private enum InitialSide {right, left}

    TeamColor teamColor;
    InitialSide initialSide;

    @Override
    public void runOpMode() {
        // init chassis
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR);
        chassis.init();

        // init arms
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Arm arm = new Arm(leftLift, rightLift, gripper);
        arm.init();
        arm.armTarget = 0;

        //OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new SleeveDetector(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    break;
                }
            }

            if(tagOfInterest != null) telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
            else telemetry.addLine("Don't see tag of interest :(");

            if(gamepad1.b || gamepad2.b) teamColor = TeamColor.red;
            if(gamepad1.x || gamepad2.x) teamColor = TeamColor.blue;
            if(gamepad1.dpad_right || gamepad2.dpad_right) initialSide = InitialSide.right;
            if(gamepad1.dpad_left || gamepad2.dpad_left) initialSide = InitialSide.left;

            telemetry.update();

            sleep(20);
        }

        boolean parked = false;

        while(opModeIsActive() && !parked) {
            arm.closeGripper();

            chassis.runToPosition(-50, -50, -50, -50);

            sleep(500);

            //arm.runToPosition(200);

            chassis.resetEncoder();

            if (tagOfInterest.id == LEFT) chassis.runToPosition(1100, -1400, -1400, 1100);
            else if (tagOfInterest.id == RIGHT) chassis.runToPosition(-1350, 1050, 1050, -1350);

            chassis.resetEncoder();

            chassis.runToPosition(-1500, -1500, -1500, -1500);

            telemetry.addLine("parked!");

            parked = true;
        }
    }
}
