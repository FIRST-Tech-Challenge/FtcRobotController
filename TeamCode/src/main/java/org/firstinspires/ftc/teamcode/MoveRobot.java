package org.firstinspires.ftc.teamcode;  //place where the code is located

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MoveRobot{
    double x;
    double y;

    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel
    
    private HardwareMap hwMap = null;  //creating objects so that they coul be mapped when initMoveRobot is called by the main programm
    private IMU imu;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    AprilTagTrackerGimbal aprilTagTrackerGimabl;
    TractionControl tractionControl;

    private boolean cameraError = false;
    private boolean imuError = false;

    private void initImu(){
        // Initializing imu to avoid errors
        imu = hardwareMap.get(IMU.class, "imu");
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

    }

    public void initMoveRobot(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        
         //mapping hardwaremap and telemetry as they need to be connected thru the main programm
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        tractionControl = new TractionControl();
        tractionControl.initTractionControl(hardwareMap, telemetry);

    	try { // init camera with safeguards
            aprilTagTrackerGimabl = new AprilTagTrackerGimbal();
            aprilTagTrackerGimabl.initAprilTag(hardwareMap, telemetry);
        } catch(Exception e) {
            cameraError=true;
        }
        

        //init imu with safeguards
        try{
            initImu();
        } catch(Exception e) {
            imuError=true;
        }
        
        // Mapping motors
        rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
        leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
    }

    // a test to return the apriltag(s) position for testing
    private void testApril(){
        if (!cameraError){
        aprilTagTrackerGimabl.telemetryAprilTag();
        }
    }

    // the main funrion for moving the robot
    public void move(double drive, double strafe, double turn, boolean fieldCentric, boolean tractionControlToggle) {
        
        if (fieldCentric && !imuError) {
            try{
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            x = drive * Math.cos(heading) - strafe * Math.sin(heading);
            y = drive * Math.sin(heading) + strafe * Math.cos(heading);
            } catch (Exception e){
                imuError = true;
            }
        } else {
            x = drive;
            y = strafe;
        }

        // Calculates raw power to motors
        double leftFrontPowerRaw = x + y + turn;
        double leftBackPowerRaw = x - y + turn;
        double rightFrontPowerRaw = x - y - turn;
        double rightBackPowerRaw = x + y - turn;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw)));

        double max = Math.max(maxRawPower, 1.0);
        double maxradian = 1972.92;

        // Calculate wheel speeds normalized to the wheels.
        double leftFrontRawSpeed = (leftFrontPowerRaw / max * maxradian / 1.2039);
        double leftBackRawSpeed = (leftBackPowerRaw / max * maxradian);
        double rightFrontRawSpeed = (rightFrontPowerRaw / max * maxradian / 1.2039);
        double rightBackRawSpeed = (rightBackPowerRaw / max * maxradian);
        
        // Make wheels go speed or use traction control
        if (tractionControlToggle){
            tractionControl.avoidSlip(leftBackRawSpeed, leftFrontRawSpeed, rightBackRawSpeed, rightFrontRawSpeed);

        } else{
        leftBackDriveEx.setVelocity(leftBackRawSpeed);
        leftFrontDriveEx.setVelocity(leftFrontRawSpeed);
        rightBackDriveEx.setVelocity(rightBackRawSpeed);
        rightFrontDriveEx.setVelocity(rightFrontRawSpeed);
        }

        try{
            testApril();
        } catch(Exception e){
            cameraError=true;
        }


        if (imuError){
            telemetry.addData("imu", "error");
        } else{
            try{
              telemetry.addData("imu", imu.getRobotYawPitchRollAngles());
            } catch(Exception e){
                imuError=true;
                }
        }
        telemetry.addData("cameraState", Boolean.toString(!cameraError));
    }
}
    