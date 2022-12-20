package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Point;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="Two Sensor Line Follow Test", group="drive")
public class TwoSensorLineFollow extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    TelemetryPacket packet = new TelemetryPacket();

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    RevColorSensorV3 colorSensorLeft;
    RevColorSensorV3 colorSensorRight;

    @Override
    public void runOpMode()
    {

        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");

        CAMShiftPipelinePowerPlay.DetectedObject detectedObject = null;
        Size size = null;
        Point center = null;
        int numCones;
        CAMShiftPipelinePowerPlay.ConeOrientation orientation;

        waitForStart();

        leftFrontMotor.setPower(-.25);
        rightFrontMotor.setPower(.25);
        leftRearMotor.setPower(-.25);
        rightRearMotor.setPower(.25);

        while (opModeIsActive())
        {
            telemetry.addData("Right Red Detected", colorSensorRight.red());
            telemetry.update();
            while (colorSensorRight.red() <= 5) {}
            leftFrontMotor.setPower(.5);
            rightFrontMotor.setPower(.25);
            leftRearMotor.setPower(.5);
            rightRearMotor.setPower(.25);

            while (colorSensorLeft.red() <= 5) {}
            leftFrontMotor.setPower(.25);
            rightFrontMotor.setPower(.5);
            leftRearMotor.setPower(.25);
            rightRearMotor.setPower(.5);

        }

    }
}