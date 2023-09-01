/* just telling android studio that this file is inside this directory */
package org.firstinspires.ftc.teamcode.common;

/* 
any packages we need to import for our code - these can't actually 
be found in the codebase and are instead fetched by android studio 
when you build your program 
*/
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/* every java file needs a class that matches its name! */
public class RobotContainer {
    /*
    This class is very important. It basically contains all the objects and methods that we'll use
    across different programs. It should also contain methods such as DriveForwards to reduce
     boilerplate code, but we didn't have time to do that in 2023.
     */

    /* 
    declare the motors and servo and set them to null
    note that not all of these motors may be applicable to
    your season!!
    */
    public DcMotorEx lf = null; // left front motor/wheel
    public DcMotorEx rf = null; // right front motor/wheel
    public DcMotorEx lb = null; // left back motor/wheel
    public DcMotorEx rb = null; // right back motor/wheel
    public DcMotorEx lift = null; // the motor that operates our lift
    public CRServo serv0; // the servomotor that operates our claw

    /* declare our gyro (imu) and camera */
    public BNO055IMU imu; // every control hub has a built-in gyroscope and it is initialized here
    public OpenCvCamera camera; // we bought a camera to assist in image recognition
    public Orientation angles; // we can use this to figure out the roll, pitch and yaw of the robot with the imu/gyro
    public Acceleration gravity; // figure it out, idk if it's useful

    /**
     * The initialization method used in every driveMode and
     * opMode to access our motors and sensors. Put this inside runOpMode
     * and pass to it the hardwareMap abstract.
     *
     * @param hwMap This parameter should be passed the abstract hardwareMap member in every LinearOpMode.
     */
    public void init(HardwareMap hwMap) {
        /* 
        map each motor to a variable we can use in our code - pretty self explanatory
        basically, on the control hub, we give each motor/servo a name
        using the HardwareMap parameter that you pass, the function will run .get() on 
        each motor/servo which basically connects the code to the hardware
        */
        lf = hwMap.get(DcMotorEx.class, "left_front");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rb = hwMap.get(DcMotorEx.class, "right_back");
        lift = hwMap.get(DcMotorEx.class, "lift");
        serv0 = hwMap.get(CRServo.class, "serv0");

        /* all of our gyro initialization stuff - you probably don't need to worry about this*/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        /* we use .get() on the imu as well! */
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /* find our webcam and initialize it */
        int cameraMonitorViewID = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "log920"), cameraMonitorViewID);

        /* make sure all of our motors are going the right way, changes across robots */
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        /* stop and reset the encoder */
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* run each motor using the encoder so we can get data */
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* 
        wow, you read the whole thing
        go to the teleop folder and look at BaseDriveComplete now!
        */
    }
}

