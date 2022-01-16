package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Caroseul")
public class Auto_RedCaroseul extends LinearOpMode {

    //robot parts
    private DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, motorOuttake, motorIntake;
    private CRServo duck;
    private Servo bucket;
    private BNO055IMU imu;

    //for robot motion
    private RobotClass robot;

    private DistanceSensor distsense;

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

        motorOuttake = hardwareMap.dcMotor.get("outtake");
        motorIntake = hardwareMap.dcMotor.get("intake");

        duck = hardwareMap.crservo.get("duck");
        bucket = hardwareMap.servo.get("bucket");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        distsense = hardwareMap.get(DistanceSensor.class,"distsense");

        //create robot object
        robot = new RobotClass(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, motorIntake, motorOuttake, bucket, duck, distsense, imu,this);

        //setup robot
        robot.runToPosSetupRobot();//TODO: if motors need swapping directions, go to this method in Robot_2022FF.java and change! DO NOT CHANGE IN HERE

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

        telemetry.addData("angle", robot.getAngle());
        telemetry.update();
        waitForStart();//if there is a camera error... and it crashes the program... then we need to find a way to "pause stream"

        int code = mainPipeline.getCode();//get the code before we move
        telemetry.addData("barcode value", code);
        if (code == 0) {
            telemetry.addData("assuming", "1");
        }
        telemetry.update();

        //dist 26 cm == 10 cm here 2.6cm=1cm
        //actually in INCHES!!!! will change in a bit
        //caroseul
        robot.gyroStrafeEncoder(0.5,-90,3);//to allow turning
        robot.gyroTurn(90,0.5);//direction

//turning: + is ccw, - is cw

        //-90 is forwards, intake is forwards
        //0 is right
        //90 is back
        //180 is left
//        robot.gyroStrafeEncoder(0.5,90,10);
        robot.gyroStrafeEncoder(0.5,-90,27);//2 feet+a bit more(error) to right. todo change the cm, direction
        robot.duck(-0.55);//turn on duck
        Thread.sleep(4000);//less time
        robot.duck(0);

        //to hub
        robot.gyroStrafeEncoder(0.5,90,52.5);
        robot.gyroTurn(90,0.5);
        robot.runToPosDrop(0.5, 21.5, code);
        Thread.sleep(500);

        // robot.goToDepot_Red();//default
        robot.goToWarehouse_Red(false);
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
