package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU


public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    //Vision

//
//    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    /**
//     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
//     */
//    public AprilTagProcessor aprilTag;
//
//    /**
//     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
//     */
//    public TfodProcessor tfod;
//
//    /**
//     * {@link #myVisionPortal} is the variable to store our instance of the vision portal.
//     */
//    public VisionPortal myVisionPortal;
//


    public Chassis chassis = new Chassis();
    public Intake intake = new Intake();
    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();
//    public Slider slider = new Slider();

    public Gate gate = new Gate();

    public Drone drone = new Drone();


    /* local OpMode members. */
    //Init hardware map
    HardwareMap hwMap = null;


    public ElapsedTime period = new ElapsedTime();
    //tells you how long the robot has run for


    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
        chassis.init(hwMap);
        intake.init(hwMap);
        arm.init(hwMap);
        wrist.init(hwMap);
        //slider.init(hwMap);
        gate.init(hwMap);
        drone.init(hwMap);
    }
}
