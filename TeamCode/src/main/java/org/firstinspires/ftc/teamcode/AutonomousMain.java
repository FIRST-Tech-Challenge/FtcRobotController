package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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


//@Autonomous(name = "Autonomous Main")
public class AutonomousMain extends LinearOpMode
{
    double hue;
    OpenCvCamera webcam;
    WebcamName webcam1;
    MainPipeline mainPipeline;
    double sensitivity;

    private CRServo leftConveyor, rightConveyor, intake;

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

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");

        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");

        flipper = hardwareMap.servo.get("flipper");

        //launcher
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");

        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BL");

        //intake and conveyor
        intake = hardwareMap.crservo.get("intake");
        leftConveyor = hardwareMap.crservo.get("leftConveyor");
        rightConveyor = hardwareMap.crservo.get("rightConveyor");

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Set zero power behaviors to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Create an IMURobot object that we will use to run the robot
        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,
                imu, wobbleArm, wobbleClaw, leftConveyor, rightConveyor, flipper, intake,
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

        //targetZone: 1 = A, 2 = B, 3 = C
        int targetZone = 0;

            if (mainPipeline.stackHeight < 20) {
                targetZone = 1; //A

            } else if (mainPipeline.stackHeight > 60) {
                targetZone = 3; //C

            } else {
                targetZone = 2; //B

            }

            telemetry.addData("Stack Height before case: ", mainPipeline.stackHeight);
            telemetry.addData("Stack Height: ", mainPipeline.stackHeight);
            telemetry.addData("tz: ", targetZone);
            telemetry.update();


            //robot.gyroStrafeCm(0.5, -90, 60);
        switch(targetZone){
            case 1: //A
                /*
                robot.gyroDriveCm(-.5, 200); //drive forward
                dropWobble(targetZone);
                //robot.gyroTurn(-45, .75);
                //robot.gyroDriveCm(-.75, 60);
                //odometryDriveToPos(100,100);

                robot.gyroStrafeCm(0.5, 90, 225); //strafe right(i think)
                outtakeLeft.setPower(.34);
                outtakeRight.setPower(.34);
                robot.gyroTurn(180, .25); //turn
                robot.gyroDriveCm(-.5, 30); //move backward behind line
                shootRings();
                robot.gyroDriveCm(.5, 38); //drive forward onto line
                 */

                shootPowerShot();
                //Go to Target Zone
                odometryDriveToPos(-16,83.2,-144.1);
                //Drop Wobble
                dropWobble(targetZone);
                //Shoot Powershot
                //Drive to Line
                odometryDriveToPos(-15.5,70,0);
                break;
            case 2: //B
                /*
                robot.gyroDriveCm(-.5, 255); //drive forward
                robot.gyroTurn(-45, .25); //turn right
                robot.gyroDriveCm(-.5, 45); //drive forward into B box
                //robot.gyroDriveCm(-.75, 40);
                dropWobble(targetZone);
                //robot.gyroTurn(30, 0.75);
                //robot.gyroDriveCm(.75, 75);
                robot.gyroDriveCm(.5, 20); //backward to line
                robot.gyroTurn(-90, .25); //turn right
                robot.gyroDriveCm(-.5, 50); //forward
                robot.gyroTurn(-45, .25); //turn right
                robot.gyroDriveCm(-.5, 50); //go forward
                robot.gyroStrafeCm(0.5, -90, 140); //strafe left
                robot.gyroDriveCm(-.5, 6); //drive forward
                outtakeLeft.setPower(.34);
                outtakeRight.setPower(.34);
                robot.gyroTurn(10, .25); //turn towards goal
                shootRings();
                //odometryDriveToPos(100,100);
                robot.gyroDriveCm(.5, 40); //drive onto line
                 */

                shootPowerShot();
                //Go to Target Zone
                odometryDriveToPos(-31.5,99.1,163);
                //Drop Wobble
                dropWobble(targetZone);
                //Shoot Powershot
                //Drive to Line
                odometryDriveToPos(-15.5,70,0);
                break;
            case 3: //C
                /*
                robot.gyroDriveCm(-.5, 344); //drive forward
                dropWobble(targetZone);
                robot.gyroDriveCm(.5, 178); //drive backward to line
                //odometryDriveToPos(100,100);
                robot.gyroStrafeCm(0.5, 90, 255); //strafe right to goal
                outtakeLeft.setPower(.34);
                outtakeRight.setPower(.34);
                robot.gyroTurn(180, .25); //turn left
                shootRings();
                robot.gyroDriveCm( .5, 40); //move onto line
                 */

                shootPowerShot();
                //Go to Target Zone
                odometryDriveToPos(-29.64,132.0,208);
                //Drop Wobble
                dropWobble(targetZone);
                //Shoot Powershot
                //Drive to Line
                odometryDriveToPos(-15.5,70,0);
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

                stackHeight = ylargestRect.height;
            }


            return output;
        }

    }


    public void shootPowerShot() throws InterruptedException{
        odometryDriveToPos(0,70.2,0);
        setOdometryAngle(0);
        //Shot 1
        odometryDriveToPos(-39.85,62.9,0);
        robot.shootRingsPower();
        //Shot 2
        odometryDriveToPos(-50.7,49.0,0);
        robot.shootRingsPower();
        //Shot 3
        odometryDriveToPos(-39.8,62.8,0);
        robot.shootRingsPower();
    }

    public void shootGoal() throws InterruptedException{
        odometryDriveToPos(0,70.2,0);
        odometryDriveToPos(-15.5,67.9,0);
        robot.shootRings();
    }

    public void odometryDriveToPos (double xPos, double yPos, double direction) {
        setOdometryAngle(0);
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);//0
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);//0
        double angle = Math.atan2(distanceY,distanceX)-(Math.PI/4);
        double distance = Math.hypot(distanceX,distanceY);//0

        double powerOne = 0.7 * Math.sin(angle);//all be 0.4
        double powerTwo = 0.7 * Math.cos(angle);//same here

        while (distance > 3){
            if (gamepad1.y){
                break;
            }
            distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            distance = Math.hypot(distanceX,distanceY);

            angle = Math.atan2(distanceY,distanceX)-(Math.PI/4);
            powerOne = 0.7 * Math.sin(angle);//all be 0.4
            powerTwo = 0.7 * Math.cos(angle);//same here

            motorFrontLeft.setPower(powerOne);
            motorFrontRight.setPower(powerTwo);
            motorBackLeft.setPower(powerTwo);
            motorBackRight.setPower(powerOne);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("DistanceX: ", distanceX);
            telemetry.addData("DistanceY: ", distanceY);
            telemetry.update();
        }
        robot.completeStop();
        setOdometryAngle(direction);
    }

    public void setOdometryAngle(double desiredAngle) {

        double angleDifference = getOdometryAngleDifference(desiredAngle);

        while (angleDifference > 3){
            if (gamepad1.y){
                break;
            }
            angleDifference = getOdometryAngleDifference(desiredAngle);
            if (angleDifference >= 15){
                if (globalPositionUpdate.returnOrientation() < desiredAngle + 180){
                    motorFrontLeft.setPower(-0.7);
                    motorBackLeft.setPower(-0.7);
                    motorFrontRight.setPower(0.7);
                    motorBackRight.setPower(0.7);
                }else{
                    motorFrontLeft.setPower(0.7);
                    motorBackLeft.setPower(0.7);
                    motorFrontRight.setPower(-0.7);
                    motorBackRight.setPower(-0.7);
                }
            }else if (angleDifference < 15 || angleDifference >= 6){
                if (globalPositionUpdate.returnOrientation() < desiredAngle + 180){
                    motorFrontLeft.setPower(-0.3);
                    motorBackLeft.setPower(-0.3);
                    motorFrontRight.setPower(0.3);
                    motorBackRight.setPower(0.3);
                }else{
                    motorFrontLeft.setPower(0.3);
                    motorBackLeft.setPower(0.3);
                    motorFrontRight.setPower(-0.3);
                    motorBackRight.setPower(-0.3);
                }
            }else if (angleDifference < 6){
                if (globalPositionUpdate.returnOrientation() < desiredAngle + 180){
                    motorFrontLeft.setPower(-0.2);
                    motorBackLeft.setPower(-0.2);
                    motorFrontRight.setPower(0.2);
                    motorBackRight.setPower(0.2);
                }else{
                    motorFrontLeft.setPower(0.2);
                    motorBackLeft.setPower(0.2);
                    motorFrontRight.setPower(-0.2);
                    motorBackRight.setPower(-0.2);
                }
            }
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
        robot.completeStop();
    }

    public double getOdometryAngleDifference(double desiredAngle){
        double angleDifference = Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());
        if (angleDifference > 180){
            angleDifference = 360 - angleDifference;
        }
        return angleDifference;
    }

    public void dropWobble(int targetZone){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < 1750){
            wobbleArm.setPower(-.3);
        }
        wobbleArm.setPower(0);

        wobbleClaw.setPosition(1);

        if(targetZone > 1) {
            timer.reset();
            while (timer.milliseconds() < 750) {
                wobbleArm.setPower(.4);
            }
            wobbleArm.setPower(0);
        }
    }


}