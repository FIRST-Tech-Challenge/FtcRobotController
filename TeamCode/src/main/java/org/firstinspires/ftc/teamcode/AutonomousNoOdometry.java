package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutonomousNoOdometry")
public class AutonomousNoOdometry extends LinearOpMode {
    double hue;
    OpenCvCamera webcam;
    WebcamName webcam1;
    MainPipeline mainPipeline;
    double sensitivity;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private DcMotor wobbleArm;
    private Servo wobbleClaw;

    private Servo flipper;

    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;

    //Figures for Odometry
    final double COUNTS_PER_INCH = 307.699557;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private DcMotor intake;

    //Declare imu
    private BNO055IMU imu;

    private IMURobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");

        flipper = hardwareMap.servo.get("flipper");

        //launcher
        /*outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");*/
        outtakeLeft=hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight=hardwareMap.get(DcMotor.class, "outtakeRight");
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //verticalLeft = hardwareMap.dcMotor.get("FL");
        //verticalRight = hardwareMap.dcMotor.get("FR");
        //horizontal = hardwareMap.dcMotor.get("BL");
        horizontal = hardwareMap.dcMotor.get("outtakeRight");
        verticalLeft = hardwareMap.dcMotor.get("wobbleArm");
        verticalRight = hardwareMap.dcMotor.get("intake");


        intake = hardwareMap.dcMotor.get("intake");


        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Set zero power behaviors to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Create an IMURobot object that we will use to run the robot
        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,
                imu, wobbleArm, wobbleClaw, flipper, intake,
                outtakeRight, outtakeLeft, this);
        robot.setupRobot();//calibrate IMU, set any required parameters

        wobbleClaw.setPosition(0);
        flipper.setPosition(1);

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        webcam.openCameraDevice();

        mainPipeline = new MainPipeline();
        webcam.setPipeline(mainPipeline);


        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        //Input Upright Mid Point: 240,320
        //Input Sideways Mid Point: 320,240

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        goShoot();

        //targetZone: 1 = A, 2 = B, 3 = C
        int targetZone = 0;
        int stackThreshold = 60;


        int stack = mainPipeline.ycontours.size();
        telemetry.addData("Stack Height before case: ", mainPipeline.stackHeight);

        if (mainPipeline.stackHeight < 70) {
            targetZone = 1;

        } else if (mainPipeline.stackHeight > 120) {
            targetZone = 3;

        } else {
            targetZone = 2;

        }

        telemetry.addData("Stack Height: ", mainPipeline.stackHeight);
        telemetry.addData("tz: ", targetZone);
        telemetry.update();



        switch(targetZone){
            case 1:
                robot.gyroDriveCm(0.5, 60);
                robot.gyroTurn(-85, 0.5);
                robot.gyroDriveCm(-0.5, 40);
                dropWobble();
                robot.gyroDriveCm(0.75,45);
                //backup
                //robot.gyroDriveCm(-.5, 10);
                //odometryDriveToPos(100,100);
                break;
            case 2:
                intake.setPower(-0.85);
                robot.gyroDriveCm(-.6, 150);
                robot.gyroDriveCm(.6, 150);
                outtakeLeft.setPower(.65);
                Thread.sleep(1500);
                intake.setPower(0);
                flipper.setPosition(0);
                Thread.sleep(500);//CHANGE!!!!!!! slower
                flipper.setPosition(1);
                Thread.sleep(500);//CHANGE!!!!!!!!
                outtakeLeft.setPower(0);
                outtakeRight.setPower(0);
                robot.gyroTurn(165, 0.5);
                robot.gyroDriveCm(-.75, 80);
                dropWobble();
                robot.gyroDriveCm(.75, 50);
                //odometryDriveToPos(100,100);
                break;
            case 3:
                intake.setPower(-0.85);
                robot.gyroDriveCm(-.6, 117);
                Thread.sleep(400);
                robot.gyroDriveCm(.6, 117);
                outtakeLeft.setPower(.65);
                Thread.sleep(2250);
                intake.setPower(0);
                flipper.setPosition(0);
                Thread.sleep(500);//CHANGE!!!!!!! slower
                flipper.setPosition(1);
                Thread.sleep(500);//CHANGE!!!!!!!!
                intake.setPower(0);
                flipper.setPosition(0);
                Thread.sleep(500);//CHANGE!!!!!!! slower
                flipper.setPosition(1);
                Thread.sleep(500);//CHANGE!!!!!!!!
                outtakeLeft.setPower(0);
                outtakeRight.setPower(0);
                robot.gyroTurn(180, 0.5);
                //robot.gyroStrafeCm(0.5, -90,80);
                robot.gyroDriveCm(-0.75, 170);
                dropWobble();
                robot.gyroDriveCm(.75, 130);
                //odometryDriveToPos(100,100);
                break;
            default:
                break;
        }

        globalPositionUpdate.stop();

    }

    class MainPipeline extends OpenCvPipeline
    {
        List<MatOfPoint> ycontours = new ArrayList<>();

        int stackHeight;

        Mat hsvImage = new Mat();
        Mat blurImg = new Mat();
        Mat output = new Mat();
        Mat yellow = new Mat();
        Mat hierachy = new Mat();


        @Override
        public Mat processFrame(Mat input) {

            input.copyTo(output);

            ycontours.clear();

            //yellow = 60
            //Blue = 240
            //red = 0 or 360
            hue = 33;
            sensitivity = 20;


            //blur image
            Imgproc.GaussianBlur(input, blurImg, new Size(5, 5), 0);

            //converting blurred image from BGR to HSV
            Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_RGB2HSV);

            //find yellow contours
            Core.inRange(hsvImage, new Scalar((hue / 2) - sensitivity, 100, 50), new Scalar((hue / 2) + sensitivity, 255, 255), yellow);
            Imgproc.findContours(yellow, ycontours, hierachy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            if (ycontours.size() > 0){
                double ymaxVal = 0;
                int ymaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < ycontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(ycontours.get(contourIdx));
                    if (ymaxVal < contourArea) {
                        ymaxVal = contourArea;
                        ymaxValIdx = contourIdx;
                    }
                }
                //Find the bounding box of the largest yellow contour
                Rect ylargestRect = Imgproc.boundingRect(ycontours.get(ymaxValIdx));
                Imgproc.rectangle(output, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 255, 255), -1, 8, 0);

                stackHeight = ylargestRect.height;
            }


            return output;
        }

    }
    public void goShoot() throws InterruptedException{
        double power = .64;
        double power_off = 0;

        outtakeLeft.setPower(power);//or 0.44
        outtakeRight.setPower(power_off);//or 0.44
        //robot.gyroStrafeCm(0.5, 90, 60);//speed up later
        robot.gyroDriveCm(0.5, 195);
        robot.gyroStrafeCm(0.5, -90, 105);

        for(int i = 0; i < 3; i++){
            Thread.sleep(500);
            flipper.setPosition(0);
            Thread.sleep(500);//CHANGE!!!!!!! slower
            flipper.setPosition(1);
            Thread.sleep(500);//CHANGE!!!!!!!!
            outtakeLeft.setPower(power);//or 0.44
            outtakeRight.setPower(power_off);//or 0.44
        }
        flipper.setPosition(1);
        outtakeLeft.setPower(power_off);
        outtakeRight.setPower(power_off);
    }

    public void robotStrafe (double power, double angle){
        //restart angle tracking
        robot.resetAngle();

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        //while(opMode.opModeIsActive()){
        //Get a correction
        double correction = robot.getCorrection();
        //Use the correction to adjust robot power so robot faces straight
        robot.correctedTankStrafe(leftPower, rightPower, correction);
        //}
    }

    public void dropWobble(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < 1500){
            wobbleArm.setPower(-.5);
        }
        wobbleArm.setPower(0);

        wobbleClaw.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 1500) {
            wobbleArm.setPower(.5);
        }
        wobbleArm.setPower(0);

    }

}