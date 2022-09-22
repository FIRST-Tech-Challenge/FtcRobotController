package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class StateCollectFreight implements EbotsAutonState{


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;
    OpenCvCamera camera;
    ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
    ArrayList<DcMotorEx> rightMotors = new ArrayList<>();
    HardwareMap hardwareMap;
    long stateTimeLimit = 5000;
    StopWatch stopWatch = new StopWatch();
    String logTag = "EBOTS";
    private final Intake intake;
    FreightDetector freightDetector;
    boolean freightPresent = false;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateCollectFreight(EbotsAutonOpMode autonOpMode) {
        this.autonOpMode = autonOpMode;
        hardwareMap = autonOpMode.hardwareMap;
        this.telemetry = autonOpMode.telemetry;
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        rightMotors.add(frontRight);
        leftMotors.add(frontLeft);
        rightMotors.add(backRight);
        leftMotors.add(backLeft);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = Intake.getInstance(hardwareMap);

        Bucket bucket = Bucket.getInstance(autonOpMode);
        bucket.setState(BucketState.COLLECT);
        freightDetector = new FreightDetector();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d(logTag, "cameraMonitorViewId set");
        EbotsWebcam bucketWebCam = new EbotsWebcam(hardwareMap, "bucketCam", RobotSide.FRONT, 0,-3.25f, 9.0f);
        WebcamName webcamName = bucketWebCam.getWebcamName();
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        Log.d(logTag, "camera instantiated");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d(logTag, "The camera is now open...");
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(freightDetector);

            }
            @Override
            public void onError(int errorCode)
            {
                Log.d(logTag, "There was an error");
            }
        });
        Log.d(logTag, "Camera for Freight Detector Instantiated");

    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private void updateFreightPresent(){
        if (!freightDetector.isReadingConsumed()) {
            double confidenceThreshold = 0.8;

            if (freightDetector.getIsBall() | freightDetector.getIsBox()){
                freightPresent = true;
            } else {
                freightPresent = false;
            }
            freightDetector.markReadingAsConsumed();
        }
        telemetry.addData("Box Present", freightDetector.getIsBox());
        telemetry.addData("Ball Present", freightDetector.getIsBall());
    }

    @Override
    public boolean shouldExit() {
        //stopping intake motor
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;
        boolean isFreightCollected = false;
        //Exit if the camera reads non-red values with in the frame
        updateFreightPresent();
        if(freightPresent) Log.d(logTag, "Freight detected during StateCollectFreight");

        return  stateTimedOut | autonOpMode.isStarted() | autonOpMode.isStopRequested() | freightPresent;
    }

    @Override
    public void performStateActions() {
            intake.start();
            for(DcMotorEx m : leftMotors) {
                m.setPower(0.15);
            }
            for(DcMotorEx m : rightMotors) {
                m.setPower(0.15);
        }
    }

    @Override
    public void performTransitionalActions() {
        try {
            camera.stopStreaming();
        } catch (Exception e){
            Log.d(logTag, "There was an exception when shutting down the camera");
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(0.0);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(0.0);
        }
        intake.stop();
    }
}