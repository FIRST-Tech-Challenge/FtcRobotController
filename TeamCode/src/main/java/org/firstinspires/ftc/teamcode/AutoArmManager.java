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

public class AutoArmManager {

    public static final double ARM_SPEED = 0.1;
    public static final double ARM_MIN = 0.10 ;
    public static final double ARM_MAX = 0.60;

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    public AutoArmManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void MoveArmToPosition( double position){
        hornetRobo.ArmOne.setPosition(Range.clip(position, ARM_MIN, ARM_MAX));
        hornetRobo.ArmTwo.setPosition(Range.clip(position, ARM_MIN, ARM_MAX));
    }

}


