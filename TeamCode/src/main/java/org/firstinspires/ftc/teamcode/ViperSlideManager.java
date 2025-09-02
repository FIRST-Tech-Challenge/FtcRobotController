/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ViperSlideManager {

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    /*TODO: Switch to viper slide encoder resolution, gear ratio, wheel diameter*/
    static final double     DRIVE_MOTOR_ENCODER_RESOLUTION = 384.5;
    static final double     DRIVE_MOTOR_GEAR_RATIO = 1;
    static final double     COUNTS_PER_MOTOR_REV    = DRIVE_MOTOR_ENCODER_RESOLUTION * DRIVE_MOTOR_GEAR_RATIO ;
    static final double     WHEEL_DIAMETER_INCHES   = 1.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ViperSlideManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
    }

    public void SetMotorsMode(DcMotor.RunMode MotorRunMode) {
        hornetRobo.ViperSlideOne.setMode(MotorRunMode);
        hornetRobo.ViperSlideTwo.setMode(MotorRunMode);
    }

    public void SetDirection(DriveManager.DriveDirection Direction){
        if (Direction == DriveManager.DriveDirection.FORWARD)
        {
            hornetRobo.ViperSlideOne.setDirection(DcMotor.Direction.FORWARD);
            hornetRobo.ViperSlideTwo.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (Direction == DriveManager.DriveDirection.BACKWARD)
        {
            hornetRobo.ViperSlideOne.setDirection(DcMotor.Direction.REVERSE);
            hornetRobo.ViperSlideTwo.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void SetTargetPosition (double DistanceInInches){

        int targetPosition = GetEncodedDistance(DistanceInInches);
        hornetRobo.ViperSlideOne.setTargetPosition(targetPosition);
        hornetRobo.ViperSlideTwo.setTargetPosition(targetPosition);
    }

    public void SetPower (Double Power){
        hornetRobo.ViperSlideOne.setPower(Power);
        hornetRobo.ViperSlideTwo.setPower(Power);
    }

    public int GetEncodedDistance(double DistanceInInches)
    {
        int encodedDistance = (int)(DistanceInInches * COUNTS_PER_INCH);
        return encodedDistance;
    }

    public void ResetAndSetToEncoder()
    {
        SetMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveStraightToPosition (DriveManager.DriveDirection DirectionToMove, double Speed, int DistanceToMove) {
        ResetAndSetToEncoder();
        SetTargetPosition(DistanceToMove);
        SetPower(Speed);
        SetMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        WaitForMotorToDoTheTask();
    }

    public void WaitForMotorToDoTheTask()
    {
        while (hornetRobo.ViperSlideOne.isBusy() || hornetRobo.ViperSlideTwo.isBusy())
        {
            ;
        }
    }

    public void BrakeOrReleaseViperSlide(Boolean Brake)
    {
        if (Brake) {
            hornetRobo.ViperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hornetRobo.ViperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            hornetRobo.ViperSlideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hornetRobo.ViperSlideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }
    }
}


