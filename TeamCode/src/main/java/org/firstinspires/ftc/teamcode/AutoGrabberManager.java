/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutoGrabberManager {

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    public static final double GRABBER_MIN = 0.10 ;
    public static final double GRABBER_MAX = 0.40;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutoGrabberManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void OpenOrCloseGrabber(boolean Open){
        double grabberPosition = Open? GRABBER_MAX: GRABBER_MIN;
        hornetRobo.Grabber.setPosition(grabberPosition);
    }
}


