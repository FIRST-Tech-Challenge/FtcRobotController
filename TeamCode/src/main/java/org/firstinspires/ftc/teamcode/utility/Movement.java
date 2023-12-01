package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This class contains methods to control drive base movement
 */
public class Movement {

    public int motor_ticks;
    public static enum Direction {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }
    private DcMotor lfDrive;
    private DcMotor rfDrive;
    private DcMotor lbDrive;
    private DcMotor rbDrive;
    private IMU imu;

    private Telemetry telemetry;

    // Tracks if it's quicker to turn right or left
    double turnError = 0;

    /**
     * Pulls in information about the motors that is determined during initialization and makes
     * that information accessible to the rest of the class.
     * @param leftFrontDrive  the front left wheels motor,
     * @param  rightFrontDrive  the front right wheels motor,
     * @param  leftBackDrive  the back left wheels motor,
     * @param  rightBackDrive  the back right wheels motor
     */
    public Movement(DcMotor leftFrontDrive, DcMotor rightFrontDrive,
                    DcMotor leftBackDrive, DcMotor rightBackDrive, IMU imu1, Telemetry telemetry1){
        lfDrive = leftFrontDrive;
        rfDrive = rightFrontDrive;
        lbDrive = leftBackDrive;
        rbDrive = rightBackDrive;
        imu = imu1;
        telemetry = telemetry1;
    }

    public void initIMU () {
        // Initialize the IMU.
        // For Tiny - Logo UP; USB BACKWARD
        // For Rosie - Logo BACKWARD; USB UP
        // For Gge - Logo UP; USB FORWARD;
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    /**
     * Resets all wheel motor encoder positions to 0
     */
    private void initMovement(){
        motor_ticks = 0;
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        //rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
    }

    /**
     * Moves all wheel motors forward a distance in ticks
     * @param power  the power given to the motors
     */
    public void Forward(int ticks, double power){
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
        }
    }

    /**
     * Moves all wheel motors backwards a distance in ticks
     * @param power  the power given to the motors
     */
    public void Backwards(int ticks, double power){
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
        }
    }

    /**
     * Moves the front right and the back left motor forwards, and the front left and the back right
     * motors backwards in order to move left a distance in ticks
     * 
     * @param power  the power given to the motors
     */
    public void Left(int ticks, double power){
        initMovement();
        lfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
        }
    }

    /**
     * Moves the front left and the back right motor forwards, and the front right and the back left
     * motors backwards in order to move left a distance in ticks
     * @param power  the power given to the motors
     */
    public void Right(int ticks, double power){
        initMovement();
        lfDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        rfDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        lbDrive.setTargetPosition(ticks * -1); // Tells the motor that the position it should go to is desiredPosition
        rbDrive.setTargetPosition(ticks); // Tells the motor that the position it should go to is desiredPosition
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setPower(power);
        rfDrive.setPower(power);
        lbDrive.setPower(power);
        rbDrive.setPower(power);
        // Hold the start of the next command until this movement is within 30 ticks of its position
        while(abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) > 30){
        }
    }

    /**
     * turns the robot in place a distance in degrees
     * @param degrees - the distance of the rotation in degrees
     */
    public void Rotate(double degrees){

        double currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        turnError = degrees + 2.5; // This works starting with +2.5 degrees (possibly inertia)

        // Through measurement, Gge takes 1500 ticks to complete a full rotation
        // Set the initial stepSize to what we expect the rotation to need
        int stepSize = (int) (turnError / 360 * 1500);

        // Move slowly and in increments
        //Move the right front motor forward
        rfDrive.setTargetPosition(rfDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left front motor backwards
        lfDrive.setTargetPosition(lfDrive.getCurrentPosition() + (stepSize));
        // Move the right back motor forward
        rbDrive.setTargetPosition(rbDrive.getCurrentPosition() + (stepSize * -1));
        // Move the left back motor backward
        lbDrive.setTargetPosition(lbDrive.getCurrentPosition() + (stepSize));

        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfDrive.setPower(0.5);
        lfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        lbDrive.setPower(0.5);

        currentDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turnError = degrees - currentDirection;
        //Closed loop turn.  Stay in the while loop until the desired bering is achieved.
        while (abs (lbDrive.getTargetPosition() - lbDrive.getCurrentPosition()) < 10) {

        }
    }

    /**
     * Calculates the turn error and contains it within 180 and -180
     * @param desiredDirection - which direction you want to go to
     * @param currentDirection - your current direction
     */
    public double CalcTurnError(double desiredDirection, double currentDirection){
        turnError = desiredDirection - currentDirection;
        if (turnError < -180) {
            turnError = turnError + 360;
        } else if (turnError > 180) {
            turnError = turnError - 360;
        }
        return turnError;
    }
}
