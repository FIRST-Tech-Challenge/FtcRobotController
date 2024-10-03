package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A default Mecanum Drive
 * Contains drive train code and imu orientation
 */
public class Mecanum {

    //CONSTRAINTS
    public final double MAX_POWER = 1;

    //Motors and Sensors
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private IMU imu;

    //Internal Variables
    private boolean fieldCentricActive;

    /**
     * Constructor for a default Mecanum Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors and sensors.
     */
    public Mecanum(HardwareMap hw){
        //Initialize Motors
        frontLeft = hw.get(DcMotor.class,"FLM");
        backLeft = hw.get(DcMotor.class,"BLM");
        frontRight = hw.get(DcMotor.class,"FRM");
        backRight = hw.get(DcMotor.class,"BRM");

        //Reverse some motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection((DcMotorSimple.Direction.REVERSE));

        resetEncoders();

        //Initialize imu
        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        resetIMU();
        //Assign default variables
        fieldCentricActive = false;
    }


    /**
     * Drives the robot by powering the motors
     * @param forward [double] Power to go forward
     * @param strafe [double] Power to go sideways
     * @param rotate [double] Power to rotate
     */
    public void drive( double forward, double strafe, double rotate){
        if(fieldCentricActive) {
            double currentRotation = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double temp = forward * Math.cos(currentRotation) + strafe * Math.sin(currentRotation);
            strafe = -forward * Math.sin(currentRotation) + strafe * Math.cos(currentRotation);
            forward = temp;
        }
        strafe *= -1;
        rotate *= -1;
        frontLeft.setPower((forward + strafe + rotate)*MAX_POWER);
        backLeft.setPower((forward - strafe + rotate)*MAX_POWER);
        frontRight.setPower((forward - strafe - rotate)*MAX_POWER);
        backRight.setPower((forward + strafe - rotate)*MAX_POWER);
    }

    /**
     * Resets robot orientation
     */
    public void resetIMU(){
        imu.resetYaw();
    }

    /**
     * Resets all drive encoders
     */
    public void resetEncoders(){
        //Motor/Deadwheel Encoders Initialization
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Toggles Field Centric Algorithm
     */
    public void toggleFieldCentric(){
        fieldCentricActive = !fieldCentricActive;
    }



    /**
     * Returns the current orientation of the robot in RADIANS
     * @return [double] Current Orientation of robot
     */
    public double getYaw(){ return getYaw(AngleUnit.RADIANS);}

    /**
     * Returns the current orientation of the robot in the DEGREES or RADIANS
     * @Param unit [AngleUnit] The desired angle unit type to be returned.
     * @return [double] Current Orientation of robot
     */
    public double getYaw(AngleUnit unit){
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
}
