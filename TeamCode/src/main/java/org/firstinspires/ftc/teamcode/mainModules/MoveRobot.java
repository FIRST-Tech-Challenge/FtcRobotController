package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MoveRobot {
    double x;
    double y;
    Gimbal gimbal;
    TractionControl tractionControl;
    double leftFrontRawSpeed;
    double leftBackRawSpeed;
    double rightFrontRawSpeed;
    double rightBackRawSpeed;
    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel
    private IMU imu;
    private HardwareMap hardwareMap; //creating objects so that they could be mapped when initMoveRobot is called by the main program
    private Telemetry telemetry;
    private boolean cameraError = false;
    private boolean cameraInitError = false;

    private boolean imuError = false;
    private boolean imuInitError = false;

    private void initImu() {
        try {
            // Initializing imu to avoid errors
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
            imuInitError = false;
        } catch (Exception e) {
            imuInitError = true;
        }
    }

    private void initCamera() {

        try { // init camera with safeguards
            gimbal = new Gimbal();
            gimbal.initAprilTag(hardwareMap, telemetry);
            cameraInitError = false;
        } catch (Exception e) {
            cameraInitError = true;
        }
    }

    public void initMoveRobot(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {

        //mapping hardwareMap and telemetry as they need to be connected thru the main program
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        tractionControl = new TractionControl();
        tractionControl.initTractionControl(hardwareMap, telemetry);

        initCamera();

        //init imu with safeguards
        initImu();

        // Mapping motors
        rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
        leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
        leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
        rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

        leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // a test to return the apriltag(s) position for testing
    private void testApril() {
        try {
            gimbal.telemetryAprilTag();
            cameraError = false;
        } catch (Exception e) {
            cameraError = true;
        }
    }
        // the main function for moving the robot
    public void move ( double drive, double strafe, double turn, boolean fieldCentric,
        boolean tractionControlToggle, boolean cameraToggle){

            {
                if (fieldCentric) {
                    try {
                        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                        y = drive * Math.sin(heading) + strafe * Math.cos(heading);
                        imuError = false;
                    } catch (Exception e) {
                        imuError = true;
                        x = drive;
                        y = strafe;
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

        /*telemetry.addData("leftBackPWR", leftBackPowerRaw);
        telemetry.addData("leftFrontPWR", leftFrontPowerRaw);
        telemetry.addData("rightBackPWR", rightBackPowerRaw);
        telemetry.addData("rightFrontPWR", rightFrontPowerRaw);*/

                // Calculate the maximum absolute power value for normalization
                double maxRawPower = Math.max(Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                        Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw)));

                double max = Math.max(maxRawPower, 1.0);
                double maxRadian = 1972.92;

                // Calculate wheel speeds normalized to the wheels.
                leftFrontRawSpeed = (leftFrontPowerRaw / max * maxRadian);
                leftBackRawSpeed = (leftBackPowerRaw / max * maxRadian / 1.2039);
                rightFrontRawSpeed = (rightFrontPowerRaw / max * maxRadian / 1.2039);
                rightBackRawSpeed = (rightBackPowerRaw / max * maxRadian);
            } // move robot
            /*telemetry.addData("speedLeftBack", leftBackRawSpeed);
            telemetry.addData("speedLeftFront", leftFrontRawSpeed);*/

            // Make wheels go speed or use traction control
            if (tractionControlToggle) {
                tractionControl.avoidSlip(leftBackRawSpeed, leftFrontRawSpeed, rightBackRawSpeed, rightFrontRawSpeed);

            } else {
                leftBackDriveEx.setVelocity(leftBackRawSpeed);
                leftFrontDriveEx.setVelocity(leftFrontRawSpeed);
                rightBackDriveEx.setVelocity(rightBackRawSpeed);
                rightFrontDriveEx.setVelocity(rightFrontRawSpeed);
                /*
                telemetry.addData("leftBack", leftBackDriveEx.getVelocity());
                telemetry.addData("leftFront", leftFrontDriveEx.getVelocity());
                telemetry.addData("rightBack", rightBackDriveEx.getVelocity());
                telemetry.addData("rightFront", rightFrontDriveEx.getVelocity());*/
            }

            if (cameraInitError) {
                telemetry.addData("Camera innit error", true);
                //initCamera();
            }
            if (imuInitError) {
                telemetry.addData("Imu innit error", true);
                initImu();
            }

            if (imuError){
                telemetry.addData("imu", "error");
            } else {
                try {
                    telemetry.addData("imu", imu.getRobotYawPitchRollAngles());
                } catch (Exception e) {
                    imuError = true;
                }
            }

            if (cameraToggle) {
                if (cameraError){
                    telemetry.addData("camera Error", true);
                }
                testApril();
            }


        }
    }
