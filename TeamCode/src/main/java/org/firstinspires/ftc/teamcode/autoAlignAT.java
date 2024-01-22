package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "tagAlignment")
public class autoAlignAT extends LinearOpMode {
    int angleSensitivity= 2;
    int strafeSensitivity = 6;
    boolean Angled;
    boolean Centered;
    boolean DistanceAway;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    public void SetDistance(int distance)
    {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        if(distance < 11){
            float Dpower = 0.1f;
            frontLeft.setPower(-Dpower);
            frontRight.setPower(-Dpower);
            backLeft.setPower(-Dpower);
            backRight.setPower(-Dpower);
        } else if(distance > 12) {
            float Dpower = 0.1f;
            frontLeft.setPower(Dpower);
            frontRight.setPower(Dpower);
            backLeft.setPower(Dpower);
            backRight.setPower(Dpower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            DistanceAway = true;
        }
    }
    public void SetAngle(int AngleError)
    {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        if (AngleError > angleSensitivity) {
            float Spower = 0.1f;
            frontLeft.setPower(-Spower);
            frontRight.setPower(Spower);
            backLeft.setPower(-Spower);
            backRight.setPower(Spower);
        } else if (AngleError < -angleSensitivity) {
            float Spower = 0.1f;
            frontLeft.setPower(Spower);
            frontRight.setPower(-Spower);
            backLeft.setPower(Spower);
            backRight.setPower(-Spower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            Angled = true;
        }
    }

    public void Center(float StrafeError)
    {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        if (StrafeError < (320 - strafeSensitivity)) {
            float StPower = 0.12f;
            frontLeft.setPower(-StPower);
            frontRight.setPower(StPower);
            backLeft.setPower(StPower);
            backRight.setPower(-StPower);
        } else if (StrafeError > (320 + strafeSensitivity)) {
            float StPower = 0.12f;
            frontLeft.setPower(StPower);
            frontRight.setPower(-StPower);
            backLeft.setPower(-StPower);
            backRight.setPower(StPower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            Centered = true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);

        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setOutputUnits(INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal1.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal1.getCameraControl(GainControl.class);
        gain.setGain(255);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor1.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor1.getDetections().get(0);
                //visionPortal1.stopLiveView();
                float myFPS = visionPortal1.getFps();
                int AngleError = (int) tag.ftcPose.yaw;
                int distance = (int) tag.ftcPose.range;
                float StrafeError = (float) tag.center.x;

                if(!DistanceAway){
                    SetDistance(distance);
                }
                if (DistanceAway && !Angled) {
                    SetAngle(AngleError);
                }
                if(!Centered && Angled) {
                    Center(StrafeError);
                }

                /*telemetry.addData("center", tag.center);
                telemetry.addData("dist from tag: ", distance);
                telemetry.addData("Centered", Centered);
                telemetry.addData("Angled", Angled);
                telemetry.addData("yaw", AngleError);
                telemetry.addData("strafeError", StrafeError);
                telemetry.addData("fps", myFPS);
                telemetry.update();*/
            }
        }
    }
}
