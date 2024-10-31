/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class AutoDriveManager {

    public enum DriveDirection {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    /************Encoder parameters*****************/
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     DRIVE_MOTOR_ENCODER_RESOLUTION = 537.7;
    static final double     DRIVE_MOTOR_GEAR_RATIO = 1;
    static final double     COUNTS_PER_MOTOR_REV    = DRIVE_MOTOR_ENCODER_RESOLUTION * DRIVE_MOTOR_GEAR_RATIO ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Declare OpMode members. */
    private LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    private HornetRobo hornetRobo;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutoDriveManager(LinearOpMode LOpMode, HornetRobo HRobo) {

        opMode = LOpMode;
        hornetRobo = HRobo;
        opMode.telemetry.addData("Drive Manager initialize:", "");

    }

    public void ResetAndSetToEncoder()
    {
        SetAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetAllMotorsMode(DcMotor.RunMode MotorRunMode) {
        hornetRobo.LeftFrontMotor.setMode(MotorRunMode);
        hornetRobo.RightFrontMotor.setMode(MotorRunMode);
        hornetRobo.LeftBackMotor.setMode(MotorRunMode);
        hornetRobo.RightBackMotor.setMode(MotorRunMode);
    }

    public void SetMotorDirection(DriveDirection direction){
        if (direction == DriveDirection.BACKWARD)
        {
            hornetRobo.LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            hornetRobo.LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            hornetRobo.RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            hornetRobo.RightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else if (direction == DriveDirection.FORWARD)
        {
            hornetRobo.LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            hornetRobo.LeftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            hornetRobo.RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            hornetRobo.RightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void Move(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        hornetRobo.LeftFrontMotor.setPower(left);
        hornetRobo.LeftBackMotor.setPower(left);
        hornetRobo.RightFrontMotor.setPower(right);
        hornetRobo.RightBackMotor.setPower(right);
    }

    public int getEncodedDistance(double DistanceInInches)
    {
        int encodedDistance = (int)(DistanceInInches * COUNTS_PER_INCH);
        return encodedDistance;
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void SetDrivePower(double leftWheel, double rightWheel) {

        hornetRobo.RightFrontMotor.setPower(rightWheel);
        hornetRobo.RightBackMotor.setPower(rightWheel);
        hornetRobo.LeftFrontMotor.setPower(leftWheel);
        hornetRobo.LeftBackMotor.setPower(leftWheel);
    }

    public void StopRobo()
    {
        hornetRobo.RightFrontMotor.setPower(0);
        hornetRobo.RightBackMotor.setPower(0);
        hornetRobo.LeftFrontMotor.setPower(0);
        hornetRobo.LeftBackMotor.setPower(0);
    }

    public void SetTargetPosition (DriveDirection DirectionToMove, double Distance){
        int encodedDistance = getEncodedDistance(Distance);

        // Get current positions of the motors
        int currentLeftFront = hornetRobo.LeftFrontMotor.getCurrentPosition();
        int currentLeftBack = hornetRobo.LeftBackMotor.getCurrentPosition();
        int currentRightFront = hornetRobo.RightFrontMotor.getCurrentPosition();
        int currentRightBack = hornetRobo.RightBackMotor.getCurrentPosition();

        SetMotorDirection(DirectionToMove);

        // Calculate new target positions
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        newLeftFrontTarget = currentLeftFront + encodedDistance;
        newRightFrontTarget = currentRightFront + encodedDistance;

        newLeftBackTarget = currentLeftBack + encodedDistance;
        newRightBackTarget = currentRightBack + encodedDistance;

        hornetRobo.LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
        hornetRobo.LeftBackMotor.setTargetPosition(newLeftBackTarget);
        hornetRobo.RightFrontMotor.setTargetPosition(newRightFrontTarget);
        hornetRobo.RightBackMotor.setTargetPosition(newRightBackTarget);
    }

    public void SetPowerToStrafe (DriveDirection DirectionToMove, double StrafePower){
        double frontPower = 0;
        double backPower = 0;
        if (DirectionToMove == DriveDirection.LEFT) {
            frontPower = StrafePower;
            backPower = -StrafePower;

        }
        else if (DirectionToMove == DriveDirection.RIGHT)
        {
            frontPower = -StrafePower;
            backPower = StrafePower;
        }

        hornetRobo.LeftFrontMotor.setPower(frontPower);
        hornetRobo.LeftBackMotor.setPower(backPower);
        hornetRobo.RightFrontMotor.setPower(backPower);
        hornetRobo.RightBackMotor.setPower(frontPower);
    }

    public void StrafeToPosition (DriveDirection DirectionToStrafe, double StrafePower, int StrafeDistance) {
        ResetAndSetToEncoder();
        SetTargetPositionToStrafe(DirectionToStrafe, StrafeDistance);
        SetDrivePower(StrafePower, StrafePower);
        SetAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        WaitForMotorToDoTheTask();
    }

    public void MoveStraightToPosition (DriveDirection DirectionToMove, double Speed, int DistanceToMove) {
        ResetAndSetToEncoder();
        SetTargetPosition(DirectionToMove, DistanceToMove);
        SetDrivePower(Speed, Speed);
        SetAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        WaitForMotorToDoTheTask();
    }

    public void SetTargetPositionToStrafe (DriveDirection DirectionToMove, double StrafeDistance){
        int encodedDistance = getEncodedDistance(StrafeDistance);

        // Get current positions of the motors
        int currentLeftFront = hornetRobo.LeftFrontMotor.getCurrentPosition();
        int currentLeftBack = hornetRobo.LeftBackMotor.getCurrentPosition();
        int currentRightFront = hornetRobo.RightFrontMotor.getCurrentPosition();
        int currentRightBack = hornetRobo.RightBackMotor.getCurrentPosition();

        // Calculate new target positions
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        if (DirectionToMove == DriveDirection.LEFT) {
            newLeftFrontTarget = currentLeftFront + encodedDistance;
            newLeftBackTarget = currentLeftBack - encodedDistance;
            newRightFrontTarget = currentRightFront - encodedDistance;
            newRightBackTarget = currentRightBack + encodedDistance;
        }
        else if (DirectionToMove == DriveDirection.RIGHT)
        {
            newLeftFrontTarget = currentLeftFront - encodedDistance;
            newLeftBackTarget = currentLeftBack + encodedDistance;
            newRightFrontTarget = currentRightFront + encodedDistance;
            newRightBackTarget = currentRightBack - encodedDistance;
        }

        hornetRobo.LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
        hornetRobo.LeftBackMotor.setTargetPosition(newLeftBackTarget);
        hornetRobo.RightFrontMotor.setTargetPosition(newRightFrontTarget);
        hornetRobo.RightBackMotor.setTargetPosition(newRightBackTarget);
    }

    public void TurnTimed(double DriveSpeed, int ForHowLong) {
        hornetRobo.LeftFrontMotor.setPower(DriveSpeed);
        hornetRobo.LeftBackMotor.setPower(DriveSpeed);
        hornetRobo.RightFrontMotor.setPower(-DriveSpeed);
        hornetRobo.RightBackMotor.setPower(-DriveSpeed);
        sleep(ForHowLong);
    }

    public void TurnUsingEncoders(DriveDirection DirectionToTurn, double DriveSpeed, double targetPositionInInches) {

        int encodedDistance = getEncodedDistance(targetPositionInInches);

        SetAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get current positions of the motors
        int currentLeftFront = hornetRobo.LeftFrontMotor.getCurrentPosition();
        int currentLeftBack = hornetRobo.LeftBackMotor.getCurrentPosition();
        int currentRightFront = hornetRobo.RightFrontMotor.getCurrentPosition();
        int currentRightBack = hornetRobo.RightBackMotor.getCurrentPosition();

        // Calculate new target positions
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        if (DirectionToTurn == DriveDirection.LEFT) {
            newLeftFrontTarget = currentLeftFront + encodedDistance;
            newLeftBackTarget = currentLeftBack + encodedDistance;
            newRightFrontTarget = currentRightFront - encodedDistance;
            newRightBackTarget = currentRightBack - encodedDistance;
        }
        else if (DirectionToTurn == DriveDirection.RIGHT) {
            newLeftFrontTarget = currentLeftFront - encodedDistance;
            newLeftBackTarget = currentLeftBack - encodedDistance;
            newRightFrontTarget = currentRightFront + encodedDistance;
            newRightBackTarget = currentRightBack + encodedDistance;
        }

        hornetRobo.LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
        hornetRobo.LeftBackMotor.setTargetPosition(newLeftBackTarget);
        hornetRobo.RightFrontMotor.setTargetPosition(newRightFrontTarget);
        hornetRobo.RightBackMotor.setTargetPosition(newRightBackTarget);

        SetDrivePower(DriveSpeed, DriveSpeed);
        SetAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

        WaitForMotorToDoTheTask();

    }

    public void WaitForMotorToDoTheTask()
    {
        while (hornetRobo.LeftFrontMotor.isBusy() || hornetRobo.RightFrontMotor.isBusy()
                ||hornetRobo.LeftBackMotor.isBusy() || hornetRobo.RightBackMotor.isBusy())
        {
            ;
        }
    }
}


