package org.firstinspires.ftc.robotcontroller.previous_year_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RedCloseVision extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo wristMotor = null;
    private Servo clawMotor1 = null;
    private Servo clawMotor2 = null;
    private Servo slingshotRelease = null;
    private DcMotor hangMotor = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    OpenCvWebcam webcam1 = null;
    private examplePipeline p1;

    @Override
    public void runOpMode() {
        //initDoubleVision();

        double objectSide = 0.0;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        p1 = new examplePipeline();
        webcam1.setPipeline(p1);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        wristMotor = hardwareMap.get(Servo.class, "wrist_Motor");
        clawMotor1 = hardwareMap.get(Servo.class, "claw_Motor1");
        clawMotor2 = hardwareMap.get(Servo.class, "claw_Motor2");
        slingshotRelease = hardwareMap.get(Servo.class, "slingshot_Release");
        hangMotor = hardwareMap.get(DcMotor.class, "hang_Motor");
        //camera camera = new camera(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setDirection(Servo.Direction.REVERSE);
        clawMotor1.setDirection(Servo.Direction.FORWARD);
        clawMotor2.setDirection(Servo.Direction.REVERSE);
        slingshotRelease.setDirection(Servo.Direction.FORWARD);
        hangMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        sleep(1000);
        setDrive(0.0, 0.0, 0.0, 0.0, 0.3, true,false, 0.0);

        String objectPosition = p1.getObjectLocation();


        Double testTime = 0.0;
        while (testTime < 3000.0){
            if (objectPosition == "left"){
                telemetry.addLine("goLeft");
            }
            else if (objectPosition == "right"){
                telemetry.addLine("goRight");
            }
            else if (objectPosition == "center"){
                telemetry.addLine("goCenter");
            }
            else{
                telemetry.addLine("Problem");
            }

            telemetry.update();
            sleep(1);
            testTime += 1;
        }

        /*
        if (objectPosition == "left"){
            telemetry.addLine("goLeft");
            telemetry.addLine("LLL");
            telemetry.addLine("LLL");
            telemetry.addLine("LLL");
            telemetry.update();
            sleep(1000);
            setDrive(0.0, 0.0, 0.0, 0.9, 0.6, true,false, 0.0);
            sleep(1450);
            setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
            sleep(390);
            setDrive(0.0, 0.0, 1.0, 0.0, 0.6, true,false, 0.0);
            sleep(200);
            setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
            sleep(290);
            setDrive(0.0, 0.0, 0.0, 0.5, 0.8, true,false, 0.0);
            sleep(300);
            setDrive(0.0, 0.0, 0.0, 0.0,  0.8, false,false, 0.0);
            sleep(1500);
            setDrive(0.0, 0.0, 0.0, -1.0, 1.0, false,false, 0.0);
            sleep(1700);
            setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
            sleep(200);
            setDrive(0.0, 0.0, -1.0, 0.0, 1.0, false,false, 0.0);
            sleep(200);
            setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
            sleep(320);

            setDrive(0.0, 1.0, -0.08, 0.0, 1.0, false, false, 0.0);
            sleep(1900);
            setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);

        }
        else if (objectPosition == "right"){
            telemetry.addLine("goRight");
            telemetry.addLine("RRR");
            telemetry.addLine("RRR");
            telemetry.addLine("RRR");

            telemetry.update();
            sleep(1000);
            setDrive(0.0, 0.0, 0.0, 0.9, 0.6, true,false, 0.0);
            sleep(1450);
            setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
            sleep(390);
            setDrive(0.0, 0.0, -1.0, 0.0, 0.6, true,false, 0.0);
            sleep(200);
            setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
            sleep(290);
            setDrive(0.0, 0.0, 0.0, 0.5, 0.8, true,false, 0.0);
            sleep(300);
            setDrive(0.0, 0.0, 0.0, 0.0,  0.8, false,false, 0.0);
            sleep(1500);
            setDrive(0.0, 0.0, 0.0, -1.0, 1.0, false,false, 0.0);
            sleep(1700);
            setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
            sleep(200);
            setDrive(0.0, 0.0, 1.0, 0.0, 1.0, false,false, 0.0);
            sleep(200);
            setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
            sleep(320);

            setDrive(0.0, 1.0, -0.08, 0.0, 1.0, false, false, 0.0);
            sleep(1900);
            setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);

        }
        else if (objectPosition == "center"){
            telemetry.addLine("goCenter");
            telemetry.addLine("CCC");
            telemetry.addLine("CCC");
            telemetry.addLine("CCC");

            telemetry.update();
            setDrive(1.0, 0.0, 0.0, 0.0, 0.2, true,false, 0.0);
        }
        else{
            telemetry.addLine("goProblem");
            telemetry.addLine("PPP");
            telemetry.addLine("PPP");
            telemetry.addLine("PPP");

            telemetry.update();
        }
         */


        /*
        sleep(1000);
        setDrive(0.0, 0.0, 0.0, 0.0, 0.2, true,false, 0.0);
        sleep(1000);
        setDrive(0.0, 0.0, 0.0, 0.9, 0.5, true,false, 0.0);
        sleep(1200);
        setDrive(1.0, 0.0, 0.0, 0.0, 0.5, true,false, 0.0);
        sleep(790);
        setDrive(0.0, 0.0, 0.0, 0.5, 0.5, true,false, 0.0);
        sleep(600);
        setDrive(0.0, 0.0, 0.0, 0.0, 0.6, false,false, 0.0);
        sleep(1500);
        setDrive(0.0, 0.0, 0.0, -1.0, 0.6, false,false, 0.0);
        sleep(1700);
        setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
        sleep(650);

        setDrive(0.0, 1.0, -0.08, 0.0, 1.0, false, false, 0.0);
        sleep(1900);
        setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);
         */

        Boolean start = true;

        while (start){
            double rightValue = p1.getRightSide();
            double leftValue = p1.getLeftSide();
            double centerValue = p1.getCenterSide();
            telemetry.addData("Right: ", "%4.2f", rightValue);
            telemetry.addData("Left: ", "%4.2f", leftValue);
            telemetry.addData("Center: ", "%4.2f", centerValue);

            if (objectPosition == "left") {
                telemetry.addLine("goLeft");
            }
            else if (objectPosition == "right"){
                telemetry.addLine("goRight");
            }
            else if (objectPosition == "center"){
                telemetry.addLine("goCenter");
            }
            else{
                telemetry.addLine("Problem");
            }
        }
        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        /*
        while (start) {
            telemetry.addData("Right: ", "%4.2f", rightValue);
            telemetry.addData("Left: ", "%4.2f", leftValue);
            telemetry.addData("Center: ", "%4.2f", centerValue);

            if (objectPosition == "left"){
                telemetry.addLine("goLeft");

                sleep(1000);
                setDrive(0.0, 0.0, 0.0, 0.9, 0.6, true,false, 0.0);
                sleep(1450);
                setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
                sleep(390);
                setDrive(0.0, 0.0, 1.0, 0.0, 0.6, true,false, 0.0);
                sleep(200);
                setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
                sleep(290);
                setDrive(0.0, 0.0, 0.0, 0.5, 0.8, true,false, 0.0);
                sleep(300);
                setDrive(0.0, 0.0, 0.0, 0.0,  0.8, false,false, 0.0);
                sleep(1500);
                setDrive(0.0, 0.0, 0.0, -1.0, 1.0, false,false, 0.0);
                sleep(1700);
                setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
                sleep(200);
                setDrive(0.0, 0.0, -1.0, 0.0, 1.0, false,false, 0.0);
                sleep(200);
                setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
                sleep(320);

                setDrive(0.0, 1.0, -0.08, 0.0, 1.0, false, false, 0.0);
                sleep(1900);
                setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);
            }
            else if (objectPosition == "right"){
                telemetry.addLine("goRight");

                sleep(1000);
                setDrive(0.0, 0.0, 0.0, 0.9, 0.6, true,false, 0.0);
                sleep(1450);
                setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
                sleep(390);
                setDrive(0.0, 0.0, -1.0, 0.0, 0.6, true,false, 0.0);
                sleep(200);
                setDrive(1.0, 0.0, 0.0, 0.0, 0.6, true,false, 0.0);
                sleep(290);
                setDrive(0.0, 0.0, 0.0, 0.5, 0.8, true,false, 0.0);
                sleep(300);
                setDrive(0.0, 0.0, 0.0, 0.0,  0.8, false,false, 0.0);
                sleep(1500);
                setDrive(0.0, 0.0, 0.0, -1.0, 1.0, false,false, 0.0);
                sleep(1700);
                setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
                sleep(200);
                setDrive(0.0, 0.0, 1.0, 0.0, 1.0, false,false, 0.0);
                sleep(200);
                setDrive(-1.0, 0.0, 0.0, 0.0, 1.0, false,false, 0.0);
                sleep(320);

                setDrive(0.0, 1.0, -0.08, 0.0, 1.0, false, false, 0.0);
                sleep(1900);
                setDrive(0.0, 0.0, 0.0, 0.0, 1.0, false, false, 0.0);
            }
            else if (objectPosition == "center"){
                telemetry.addLine("goCenter");
            }
            else{
                telemetry.addLine("Problem");
            }

            start = false;
            // Push telemetry to the Driver Station.
            //telemetry.update();
        }   // end while loop
         */

    }   // end method runOpMode()


    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();

        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    private void setDrive(double setAxial, double setLateral, double setYaw, double setArmPitch, double setWristPitch, boolean setClawOpen, boolean setPlaneLaunch, double setHangStrength){
        double max;
        double wristMotorPower = 0.0;
        double clawMotorPower = 0.0;
        double slingshotPosition = 0.3;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = setAxial + setLateral + setYaw;
        double rightFrontPower = setAxial - setLateral - setYaw;
        double leftBackPower   = setAxial - setLateral + setYaw;
        double rightBackPower  = setAxial + setLateral - setYaw;
        double armMotorPower = 0.70 * setArmPitch;
        double hangMotorPower = setHangStrength;
        wristMotorPower = setWristPitch;


        if (setClawOpen) {
            clawMotorPower = 0.0;
        }
        else {
            clawMotorPower = 1.0;
        }

        if (setPlaneLaunch){
            slingshotPosition = 1.0;
        }
        else {
            slingshotPosition = 0.3;
        }

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(armMotorPower));
        max = Math.max(max, Math.abs(wristMotorPower));
        max = Math.max(max, Math.abs(clawMotorPower));
        max = Math.max(max, Math.abs(slingshotPosition));
        max = Math.max(max, Math.abs(hangMotorPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
            armMotorPower   /= max;
            wristMotorPower /= max;
            clawMotorPower  /= max;
            slingshotPosition /= max;
            hangMotorPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        armMotor.setPower(armMotorPower);
        wristMotor.setPosition(wristMotorPower);
        clawMotor1.setPosition(clawMotorPower);
        clawMotor2.setPosition(clawMotorPower);
        slingshotRelease.setPosition(slingshotPosition);
        hangMotor.setPower(hangMotorPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Arm", "%4.2f", armMotorPower);
        telemetry.addData("Wrist", "%4.2f", wristMotorPower);
        telemetry.addData("Claw", "%4.2f", clawMotorPower);
        telemetry.addData("Flywheel", "%4.2f", slingshotPosition);
        telemetry.addData("HangMotor", "%4.2f", hangMotorPower);
        telemetry.update();
    } //end setDrive
    class examplePipeline extends OpenCvPipeline {
        String objectLocation = "left";
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;
        double leftavgfin;
        double rightavgfin;
        double centeravgfin;
        Mat outPut = new Mat();
        Scalar rectColor1 = new Scalar(255.0, 0.0, 255.0);
        Scalar rectColor2 = new Scalar(0.0, 255.0, 0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 199, 359);
            Rect rightRect = new Rect(440, 1, 199, 359);
            Rect centerRect = new Rect(240, 1, 159, 359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor2, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor2, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor1, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            centerCrop = YCbCr.submat(centerRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(centerCrop, centerCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar centeravg = Core.mean(centerCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            centeravgfin = centeravg.val[0];

            if (leftavgfin > rightavgfin && leftavgfin > centeravgfin){
                telemetry.addLine("left");
                objectLocation = "left";
            }
            else if (rightavgfin > leftavgfin && rightavgfin > centeravgfin){
                telemetry.addLine("right");
                objectLocation = "right";
            }

            /*else if(centeravgfin > leftavgfin && centeravgfin > rightavgfin){
                telemetry.addLine("center");
                objectLocation = "center";
            }
             */
            return (outPut);
        }

        public String getObjectLocation(){
            return objectLocation;
        }
        public double getRightSide(){
            return rightavgfin;
        }
        public double getLeftSide(){
            return leftavgfin;
        }
        public  double getCenterSide(){
            return centeravgfin;
        }
    }
}