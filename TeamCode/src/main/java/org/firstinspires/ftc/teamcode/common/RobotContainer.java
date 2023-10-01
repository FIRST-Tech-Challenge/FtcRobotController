/* just telling android studio that this file is inside this directory */
package org.firstinspires.ftc.teamcode.common;

/* 
any packages we need to import for our code - these can't actually 
be found in the codebase and are instead fetched by android studio 
when you build your program 
*/

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx lf = null; // left front motor/wheel
    public DcMotorEx rf = null; // right front motor/wheel
    public DcMotorEx lb = null; // left back motor/wheel
    public DcMotorEx rb = null; // right back motor/wheel
    public IMU imu = null;

    /* declare our gyro (imu) and camera */

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

        /* all of our gyro initialization stuff - you probably don't need to worry about this*/
        /*
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

         */

        /* make sure all of our motors are going the right way, changes across robots */
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        /* stop and reset the encoder */
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        wow, you read the whole thing
        go to the teleop folder and look at BaseDriveComplete now!
        */
    }
}

