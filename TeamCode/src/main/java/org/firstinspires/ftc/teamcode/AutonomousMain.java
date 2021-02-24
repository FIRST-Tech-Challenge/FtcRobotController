package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Autonomous Main")
public class AutonomousMain extends LinearOpMode
{
    double hue;
    OpenCvCamera webcam;
    WebcamName webcam1;
    MainPipeline mainPipeline;
    double sensitivity;

    private DcMotor intake;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private DcMotor wobbleArm;
    private Servo wobbleClaw;

    private Servo flipper;

    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;

    final double COUNTS_PER_INCH = 307.699557;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    //Declare imu
    private BNO055IMU imu;

    private IMURobot robot;

    double threshold1;
    double threshold2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        //intake and conveyor
        intake = hardwareMap.dcMotor.get("intake");

        //wobble and flipper
        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        flipper = hardwareMap.servo.get("flipper");

        //launcher  //Feb 7 - Jeff commmented out these motor definitions
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");
        //Jeff added
        //outtakeLeft=hardwareMap.get(DcMotor.class, "outtakeLeft");
        //outtakeRight=hardwareMap.get(DcMotor.class, "outtakeRight");
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Encoders
        /*
        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BL");
         */
        horizontal = hardwareMap.dcMotor.get("outtakeRight");
        verticalLeft = hardwareMap.dcMotor.get("wobbleArm");
        verticalRight = hardwareMap.dcMotor.get("intake");


        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);


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
        telemetry.addData("Ring Stack Height: ", mainPipeline.stackHeight);

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        goShoot();

        sleep(5000);

        //targetZone: 1 = A, 2 = B, 3 = C
        int targetZone = 0;
        threshold1 = 50;
        threshold2 = 120;

            if (mainPipeline.stackHeight < threshold1) {
                targetZone = 1; //A

            } else if (mainPipeline.stackHeight > threshold2) {
                targetZone = 3; //C

            } else {
                targetZone = 2; //B

            }

            telemetry.addData("Stack Height before case: ", mainPipeline.stackHeight);
            telemetry.addData("Stack Height: ", mainPipeline.stackHeight);
            telemetry.addData("tz: ", targetZone);
            telemetry.update();

        switch(targetZone){
            case 1: //A
                robot.gyroDriveCm(0.5, 60);
                robot.gyroTurn(-85, 0.5);
                robot.gyroDriveCm(-0.5, 40);
                dropWobble();
                odometryDriveToPosCorrected(-15.88,69.8,0);
                break;
            case 2: //B
                intake.setPower(0.85);
                robot.gyroDriveCm(-.6, 100);
                robot.gyroDriveCm(.6, 100);
                outtakeLeft.setPower(.65);
                Thread.sleep(2500);
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
                odometryDriveToPosCorrected(-15.88,69.8,0);

                break;
            case 3: //C
                intake.setPower(0.85);
                robot.gyroDriveCm(-.5, 185);
                Thread.sleep(1000);
                robot.gyroDriveCm(.5, 180);
                outtakeLeft.setPower(.65);
                Thread.sleep(3000);
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
                odometryDriveToPosCorrected(-15.88,69.8,0);
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

            //find orange contours
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
                //Find the bounding box of the largest orange contour
                Rect ylargestRect = Imgproc.boundingRect(ycontours.get(ymaxValIdx));
                Imgproc.rectangle(output, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 255, 255), -1, 8, 0);

                Imgproc.line(output, new Point(0,ylargestRect.y + threshold1), new Point(640, ylargestRect.y + threshold1),new Scalar(255,255,0));
                Imgproc.line(output, new Point(0,ylargestRect.y + ((threshold1 + threshold2)/2)), new Point(640, ylargestRect.y + ((threshold1 + threshold2)/2)),new Scalar(0,255,0));
                Imgproc.line(output, new Point(0,ylargestRect.y + threshold2), new Point(640, ylargestRect.y + threshold2),new Scalar(255,255,0));

                stackHeight = ylargestRect.height;
            }
            return output;
        }

    }


    public void odometryDriveToPosCorrected (double xPos, double yPos, double direction) {
        if (getOdometryAngleDifference(direction) > 1.5){
            setOdometryAngle(0);
        }
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);//0
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);//0

        double angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
        double distance = Math.hypot(distanceX,distanceY);//0

        double powerOne = 1 * Math.sin(angle);
        double powerTwo = 1 * Math.cos(angle);

        double angleDifference = getOdometryAngleDifferenceNegative(direction);

        while (distance > 1.5){
            if (gamepad1.y){
                break;
            }

            angleDifference = getOdometryAngleDifferenceNegative(direction);
            double correction = angleDifference * 0.1;


            distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            distance = Math.hypot(distanceX,distanceY);

            angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
            if (distance >= 10){
                powerOne = 1 * Math.sin(angle);
                powerTwo = 1 * Math.cos(angle);
            }else if (distance < 10 && distance > 5){
                powerOne = 0.4 * Math.sin(angle);
                powerTwo = 0.4 * Math.cos(angle);
            }else if (distance <= 5){
                powerOne = 0.3 * Math.sin(angle);
                powerTwo = 0.3 * Math.cos(angle);
            }


            motorFrontLeft.setPower(powerOne+correction);
            motorFrontRight.setPower(powerTwo-correction);
            motorBackLeft.setPower(powerTwo+correction);
            motorBackRight.setPower(powerOne-correction);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("DistanceX: ", distanceX);
            telemetry.addData("DistanceY: ", distanceY);
            telemetry.addData("Xpos: ", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("Ypos: ", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }
        robot.completeStop();
    }

    public void setOdometryAngle(double desiredAngle) {

        double rawAngleDifference = getAngleRaw(desiredAngle);
        double relativeAngleDifference = getOdometryAngleDifference(desiredAngle);


        while (relativeAngleDifference > 1.5){
            if (gamepad1.y){
                break;
            }

            rawAngleDifference = getAngleRaw(desiredAngle);
            relativeAngleDifference = getOdometryAngleDifference(desiredAngle);


            if ((desiredAngle > globalPositionUpdate.returnOrientation()) && (rawAngleDifference > 180)){
                if (relativeAngleDifference > 15){
                    turnClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnClockwise(0.2);
                }else{
                    break;
                }
            }else if ((desiredAngle < globalPositionUpdate.returnOrientation()) && (rawAngleDifference <= 180)){
                if (relativeAngleDifference > 15){
                    turnCounterClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnCounterClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnCounterClockwise(0.2);
                }else{
                    break;
                }
            }else if ((desiredAngle < globalPositionUpdate.returnOrientation()) && (rawAngleDifference > 180)){
                if (relativeAngleDifference > 15){
                    turnClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnClockwise(0.2);
                }
                else{
                    break;
                }
            }else if ((desiredAngle > globalPositionUpdate.returnOrientation()) && (rawAngleDifference <= 180)){
                if (relativeAngleDifference > 15){
                    turnCounterClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnCounterClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnCounterClockwise(0.2);
                }else{
                    break;
                }

            }else{
                break;
            }
        }
        robot.completeStop();
    }

    public void shootPowerShot() throws InterruptedException{
        //Shot 1
        odometryDriveToPosCorrected(55.6,2.99,0);
        robot.shootRingsPower();
        //Shot 2
        odometryDriveToPosCorrected(55.6,6.6,0);
        robot.shootRingsPower();
        //Shot 3
        odometryDriveToPosCorrected(55.6,8.51,0);
        robot.shootRingsPower();
    }

    public void shootGoal() throws InterruptedException{
        odometryDriveToPosCorrected(-18,60,0);
        robot.shootRings();

    }

    public double getOdometryAngleDifference(double desiredAngle){
        double angleDifference = Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());

        if (angleDifference > 180){
            angleDifference = 360 - angleDifference;
        }

        return angleDifference;
    }

    public double getAngleRaw (double desiredAngle){
        return ((double) Math.abs(desiredAngle - globalPositionUpdate.returnOrientation()));
    }

    public void turnClockwise(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
    }
    public void turnCounterClockwise(double power){
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
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

    public double getOdometryAngleDifferenceNegative(double desiredAngle){
        double angleDifference = Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());

        if (angleDifference > 180){
            angleDifference = angleDifference - 360;
        }

        return angleDifference;
    }

    public void goShoot() throws InterruptedException{
        double power = .64;
        double power_off = 0;

        outtakeLeft.setPower(power);//or 0.44
        outtakeRight.setPower(power_off);//or 0.44
        //robot.gyroStrafeCm(0.5, 90, 60);//speed up later
        robot.gyroDriveCm(0.5, 195);
        robot.gyroStrafeCm(0.5, -90, 120);

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


}