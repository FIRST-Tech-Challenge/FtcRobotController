package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsBlinkin;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Carousel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.EbotsMotionController;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.FieldOrientedVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.MecanumDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class EbotsTeleOpV2 extends LinearOpMode {

    private EbotsMotionController motionController;
    private StopWatch lockoutStopWatch = new StopWatch();
    private StopWatch endGameStopWatch = new StopWatch();
    private Telemetry.Item zeroHeadingItem = null;
    private Intake intake;
    private Carousel carousel;
    public Bucket bucket;
    private Arm arm;
    private EbotsBlinkin ebotsBlinkin;

    private boolean endGameRumbleIssued;
    private boolean justDumped = false;
    private OpenCvCamera camera;
    private String logTag = "EBOTS";
    private FreightDetector freightDetector;
    private boolean freightLoaded = false;
    private StopWatch stopWatchFreightRumble = new StopWatch();
    private long freightRumbleTimeLimit = 1500L;



    public void setJustDumped(boolean justDumped) {
        this.justDumped = justDumped;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        endGameRumbleIssued = false;
        intake = Intake.getInstance(hardwareMap);
        carousel = Carousel.getInstance(hardwareMap);
        //Check if reset is needed
        bucket = Bucket.getInstance(this);
        bucket.setState(BucketState.COLLECT);

        Log.d(logTag, "About to call Arm.getInstance from " + this.getClass().getSimpleName());
        arm = Arm.getInstance(this);

        Log.d(logTag, "About to call UtilFuncs.initManips from " + this.getClass().getSimpleName());
        UtilFuncs.initManips(arm,carousel,this);

        ebotsBlinkin = EbotsBlinkin.getInstance(hardwareMap);
        ebotsBlinkin.lightsOn();

        motionController = EbotsMotionController.get(FieldOrientedVelocityControl.class, this);
        EbotsWebcam bucketWebCam = new EbotsWebcam(hardwareMap, "bucketCam", RobotSide.FRONT, 0,-3.25f, 9.0f);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d(logTag, "cameraMonitorViewId set");

        WebcamName webcamName = bucketWebCam.getWebcamName();
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.d(logTag, "camera instantiated");
        freightDetector = new FreightDetector();
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


        while (! this.isStarted()){
            this.handleUserInput(gamepad1);
            bucket.handleUserInput(gamepad2);
            ebotsBlinkin.handleUserInput(gamepad2);
            rumbleIfFreightPresent();
            updateTelemetry();
        }

        waitForStart();
        endGameStopWatch.reset();

        while (opModeIsActive()){

            rumbleIfEndGame();
            rumbleIfFreightPresent();
            rotateBucketIfArmAtBottom();

            this.handleUserInput(gamepad1);
            motionController.handleUserInput(gamepad1);
            intake.handleUserInput(gamepad2);
            carousel.handleUserInput(gamepad2);
            bucket.handleUserInput(gamepad2);
            ebotsBlinkin.handleUserInput(gamepad2);
            arm.handleUserInput(gamepad2);

            updateTelemetry();
        }

        ebotsBlinkin.lightsOff();
    }

    private void rumbleIfEndGame() {
        if (endGameStopWatch.getElapsedTimeSeconds() >= 89 && !endGameRumbleIssued){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            endGameRumbleIssued = true;
        }
    }

    private void rumbleIfFreightPresent(){
        //Log.d(logTag, "Inside rumbleIfFreightPresent....");
        if(bucket.getBucketState() == BucketState.COLLECT){
//            freightLoaded = freightDetector.getIsBox() | freightDetector.getIsBall();
            freightLoaded = freightDetector.getIsBox() | freightDetector.getIsBall();
            boolean freightRumbleLockedOut = stopWatchFreightRumble.getElapsedTimeMillis() < freightRumbleTimeLimit;
            if(freightLoaded && !freightRumbleLockedOut){
                gamepad1.rumble(250);
                gamepad2.rumble(250);
                freightDetector.markReadingAsConsumed();
                freightLoaded = false;
                stopWatchFreightRumble.reset();
            }
        }
    }

    private void updateTelemetry() {
        String twoDecimals = "%.2f";
        Telemetry.Item zeroHeadingLine = null;
        telemetry.addData("Motion Controller", motionController.getName());
        if(motionController instanceof FieldOrientedVelocityControl){
            ((FieldOrientedVelocityControl) motionController).addVelocitiesToTelemetry(telemetry);
        }
        telemetry.addData("Carousel Power", carousel.getPower());
        telemetry.addData("Carousel Speed", carousel.getMotorVelocity());
        telemetry.addData("Arm isAtBottom", arm.isAtBottom());
        telemetry.addData("Arm position", arm.getPosition());
        telemetry.addData("Arm is zeroed ", arm.getIsZeroed());
        telemetry.addData("Arm State ", arm.getArmState());
        telemetry.addData("Bucket State ", bucket.getBucketState());
        telemetry.addData("LED State ", ebotsBlinkin.getLedState());
        telemetry.addData("Freight Detected ", freightLoaded);
        telemetry.addData("Bucket Hue ", freightDetector.getAverageHue());
        telemetry.addData("Is Box ", freightDetector.getIsBox() + " ("
                + String.format(twoDecimals, freightDetector.getConfidenceBox()) + ")");
        telemetry.addData("Is Ball ", freightDetector.getIsBall()  + " ("
                + String.format(twoDecimals, freightDetector.getConfidenceBall()) + ")");

        telemetry.update();
    }

    private void handleUserInput(Gamepad gamepad){
        boolean lockoutActive = lockoutStopWatch.getElapsedTimeMillis() < 600;

        if (lockoutActive){
            return;
        }


        if(gamepad.left_bumper && gamepad.right_stick_button){
            if (motionController instanceof MecanumDrive){
                motionController = EbotsMotionController.get(FieldOrientedVelocityControl.class, this);
            } else if (motionController instanceof FieldOrientedVelocityControl){
                motionController = EbotsMotionController.get(FieldOrientedDrive.class, this);
            } else if (motionController instanceof FieldOrientedDrive){
                motionController = EbotsMotionController.get(MecanumDrive.class, this);
            }

            gamepad.rumble(1.0, 1.0, 400);  // 200 mSec burst on left motor.
            lockoutStopWatch.reset();
        }

    }

    private void rotateBucketIfArmAtBottom(){
        if (arm.shouldBucketCollect()) {
            bucket.setState(BucketState.COLLECT);
            ebotsBlinkin.lightsOn();
        }
    }
}

