package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Caroseul Side Test")
public class Auto_2022FF extends LinearOpMode {
    //robot parts
    private DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    private DcMotor intake;
    private CRServo duck;
    private BNO055IMU imu;

    //for robot motion
    private RobotClass robot;

    //for OpenCV
    OpenCvCamera cam;// webcam
    int width = 640;
    int height = 480;
    CVClass mainPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        //TODO Only proceed if you have done PIDCalibration and copied kp and kd values!!!!!!! if you are here and you haven't you be doing something wrong
        //setup robot parts
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        duck = hardwareMap.crservo.get("duck");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //create robot object
        robot = new RobotClass(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, intake, duck, imu, this);

        //setup robot
        robot.setupRobot(false);//TODO: if motors need swapping directions, go to this method in Robot_2022FF.java and change! DO NOT CHANGE IN HERE

        //setup camera, turn it on
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        mainPipeline = new CVClass();//create new pipeline

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
            @Override
            public void onOpened() {
                cam.setPipeline(mainPipeline);//set webcam pipeline
                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...", ":(");
                System.exit(0);
            }
        });

        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"

        telemetry.addData("Did you do PIDCalibration?", "no");
        telemetry.update();
        Thread.sleep(1234);
        System.exit(1);

        int code = mainPipeline.getCode();//get the code (duck position) before we move
        telemetry.addData("barcode value", code);
        if (code == 0) {
            telemetry.addData("assuming", "1");
        }
        telemetry.update();
        //caroseul
        robot.pidGyroStrafeIn(90, 62);//2 feet+a bit more(error) to right. todo change the cm
        robot.duck(1);//turn on duck
        Thread.sleep(3000);
        robot.duck(0);

        //to hub
        //todo: adjust these values to make it work! You only need to change cm really, and maybe flip the angle for the second strafe if it's going in the wrong direction
        robot.pidGyroStrafeIn(Math.atan2(4.0, 3.0), 152);
        robot.pidGyroStrafeIn(-90, 20);

        switch (code) {//shell for later, do not delete!!!
            case 2:
                //center, middle
                //dump bucket
                break;
            case 3:
                //right, top
                //dump bucket
                break;
            case 1:
            default:
                //left, bottom
                //error, put on bottom, do case1
                //dump bucket
                break;
        }

        //go to warehouse
        robot.pidGyroTurn(90);
        robot.gyroDriveSec(1, 3);//todo change the seconds! we don't have a sensor yet, so go with this for now
        //might add sensor here, but need to make sure we have!
        //also bruhhhh they put the picture not in the warehouse! it might have been helpful if the picture was int eh warehouse bc we might be able to use openCV to detect the wall...

        //pick up element
        //todo make this a method later, NOT NOW finish testing first
        robot.intake(1);
        robot.gyroDriveSec(0.2, 1);
        robot.gyroDriveSec(-0.2, 1);
        robot.intake(0);

        //drive back to hub
        robot.gyroDriveSec(-1, 3);//todo change the seconds

        //drop in bottom of hub
        robot.pidGyroTurn(-90);

        //park in warehouse, else park in box
        //warehouse
        robot.pidGyroTurn(90);
        robot.gyroDriveSec(1, 3);//warehouse. todo also change seconds here
        //box
        robot.pidGyroStrafeIn(90, 60);//todo change cm


    }
    /*
     * ways to score in auto:
     * BLUE CAROSEUL SIDE
     * step 1: scan
     * step 2: caroseul
     * step 3: drop off element
     * step 4: get more elemnts (against wall? over bump?)
     * step 5: put in team hub
     * step 6: park in box since alliance partner in the hub
     * */
}
