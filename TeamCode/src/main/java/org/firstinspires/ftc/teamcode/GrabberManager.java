/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class GrabberManager {

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    public static final double GRABBER_MIN = 0.10 ;
    public static final double GRABBER_MAX = 0.40;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public GrabberManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void OpenOrCloseGrabber(boolean Open){
        double grabberPosition = Open ? GRABBER_MAX : GRABBER_MIN;
        hornetRobo.Grabber.setPosition(grabberPosition);
    }

    public void SetGrabberToPosition(double Position){
        hornetRobo.Grabber.setPosition(Range.clip(Position, GRABBER_MIN, GRABBER_MAX));
    }

}


