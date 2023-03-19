package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamUtil.Angle;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class MecanumDriveBase extends Subsystem {
    RobotConfig r;
    private IMU imu;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontLeft;

    private boolean fieldCentric;
    
    private double powerX;
    private double powerY;
    private double headingX;
    private double headingY;
    private double headingDelta;
    private double robotHeadingRadians;
    private Angle targetHeading;
    private Angle robotHeading;
    private double imuOffset;
    
    private boolean flipped;
    private boolean POVmode;
    
    public MecanumDriveBase(RobotConfig r, boolean fieldCentric){
        this.r = r;
        this.fieldCentric = fieldCentric;
        frontRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontRight);
        backRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backRight);
        backLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backLeft);
        frontLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontLeft);
    
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    
    
        imu = r.opMode.hardwareMap.get(IMU.class, ConfigNames.imu);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        resetIMU();
    }
    public MecanumDriveBase(boolean fieldCentric){
        this(RobotConfig.getInstance(), fieldCentric);
    }

    public void trueHolonomicDrive(double planarX, double planarY, double headingX, double headingY, double throttle){
        
        if(fieldCentric){
            this.powerX = planarY * Math.sin(robotHeadingRadians) + planarX * Math.cos(robotHeadingRadians);
            this.powerY = planarY * Math.cos(robotHeadingRadians) - planarX * Math.sin(robotHeadingRadians);
        }
        else{
            this.powerX = planarX;
            this.powerY = planarY;
        }
        this.headingX = headingX;
        this.headingY = headingY;
        
        this.frontRightPower = (powerX + powerY + headingDelta) * throttle;
        this.backRightPower = (powerX - powerY + headingDelta) * throttle;
        this.backLeftPower = (powerX + powerY - headingDelta) * throttle;
        this.frontLeftPower = (powerX - powerY - headingDelta) * throttle;
        
    }

    public void setFieldCentric(boolean fieldCentric){
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void init() {
        setFieldCentric(fieldCentric);
        resetIMU();
    }

    @Override
    public void read() {
        if(fieldCentric){
            robotHeading = new Angle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-imuOffset+90);
            robotHeadingRadians = Math.toRadians(robotHeading.getValue());
            if(POVmode){
                headingDelta = -headingX;
            }
            else {
                if (flipped){
                    targetHeading = new Angle (Angle.atanHandler(headingX, headingY).getValue()+180);
                }
                else{
                    targetHeading = new Angle (Angle.atanHandler(headingX, headingY).getValue());
                }
                headingDelta = -(robotHeading.angleShortDifference(targetHeading)/45.0)/Math.max(1, Math.abs(headingDelta));
                if (Double.isNaN(headingDelta)){
                    headingDelta = 0;
                }
            }
        }
        else{
            headingDelta = -headingX;
        }
    }
    
    private double frontRightPower;
    private double backRightPower;
    private double backLeftPower;
    private double frontLeftPower;
    private double cachedFrontRightPower;
    private double cachedBackRightPower;
    private double cachedBackLeftPower;
    private double cachedFrontLeftPower;

    @Override
    public void update() {
        if(Math.abs(Math.abs(cachedFrontRightPower) - Math.abs(frontRightPower)) >= 0.01) {
            frontRight.setPower(frontRightPower);
            cachedFrontRightPower = frontRightPower;
        }
        if(Math.abs(Math.abs(cachedBackRightPower) - Math.abs(backRightPower)) >= 0.01) {
            backRight.setPower(backRightPower);
            cachedBackRightPower = backRightPower;
        }
        if(Math.abs(Math.abs(cachedBackLeftPower) - Math.abs(backLeftPower)) >= 0.01) {
            backLeft.setPower(backLeftPower);
            cachedBackLeftPower = backLeftPower;
        }
        if(Math.abs(Math.abs(cachedFrontLeftPower) - Math.abs(frontLeftPower)) >= 0.01) {
            frontLeft.setPower(frontLeftPower);
            cachedFrontLeftPower = frontLeftPower;
        }
    
        r.opMode.telemetry.addData("yaw", robotHeading.getValue());
        r.opMode.telemetry.addData("roll", new Angle(imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES)).getValue());
        r.opMode.telemetry.addData("pitch", new Angle(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES)).getValue());
        r.opMode.telemetry.addData("front left", cachedFrontLeftPower);
        r.opMode.telemetry.addData("front right", cachedFrontRightPower);
        r.opMode.telemetry.addData("back left", cachedBackLeftPower);
        r.opMode.telemetry.addData("back right", cachedBackRightPower);
        r.opMode.telemetry.addData("headingX", headingX);
        r.opMode.telemetry.addData("headingY", headingY);
        r.opMode.telemetry.addData("powerX", powerX);
        r.opMode.telemetry.addData("powerY", powerY);
        r.opMode.telemetry.addData("headingDelta", headingDelta);
        r.opMode.telemetry.addData("target Heading", targetHeading.getValue());
    }

    @Override
    public void close() {
    
    }
    
    public void resetIMU(){
        imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public void flipDriveBase(){
        flipped = !flipped;
    }
    
    public void POVmode(boolean POVmode){
        this.POVmode = POVmode;
    }
}