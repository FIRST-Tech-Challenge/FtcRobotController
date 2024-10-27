/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutoViperSlideManager {

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    /*TODO: Switch to viper slide encoder resolution, gear ratio, wheel diameter*/
    static final double     DRIVE_MOTOR_ENCODER_RESOLUTION = 537.7;
    static final double     DRIVE_MOTOR_GEAR_RATIO = 19.2;
    static final double     COUNTS_PER_MOTOR_REV    = DRIVE_MOTOR_ENCODER_RESOLUTION * DRIVE_MOTOR_GEAR_RATIO ;
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutoViperSlideManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void SetMotorsMode(DcMotor.RunMode MotorRunMode) {
        hornetRobo.ViperSlideOne.setMode(MotorRunMode);
        hornetRobo.ViperSlideTwo.setMode(MotorRunMode);
    }

    public void SetDirection(AutoDriveManager.DriveDirection Direction){
        if (Direction == AutoDriveManager.DriveDirection.FORWARD)
        {
            hornetRobo.ViperSlideOne.setDirection(DcMotor.Direction.FORWARD);
            hornetRobo.ViperSlideTwo.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (Direction == AutoDriveManager.DriveDirection.BACKWARD)
        {
            hornetRobo.ViperSlideOne.setDirection(DcMotor.Direction.REVERSE);
            hornetRobo.ViperSlideTwo.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void SetTargetPosition (double DistanceInInches){

        int targetPosition = getEncodedDistance(DistanceInInches);
        hornetRobo.ViperSlideOne.setTargetPosition(targetPosition);
        hornetRobo.ViperSlideTwo.setTargetPosition(targetPosition);
    }

    public void SetPower (Double Power){
        hornetRobo.ViperSlideOne.setPower(Power);
        hornetRobo.ViperSlideTwo.setPower(Power);
    }

    public int getEncodedDistance(double DistanceInInches)
    {
        int encodedDistance = (int)(DistanceInInches * COUNTS_PER_INCH);
        return encodedDistance;
    }

}


