package org.firstinspires.ftc.teamcode;  //place where the code is located

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TractionControl{

    private double lastTime = 0; // create last varriables for traction control
    private double lastSpeedLeftFrontDrive = 0;
    private double lastSpeedLeftBackDrive = 0;
    private double lastSpeedRightBackDrive = 0;
    private double lastSpeedRightFrontDrive = 0;
    private final double accelerationLimit = 20; //RPSS revolutions per second  second 

    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public void initTractionControl(HardwareMap hardwareMapPorted, Telemetry telemetryPorted){
        
        hardwareMap=hardwareMapPorted;
        telemetry=telemetryPorted;

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

    public void avoidSlip(double speedLeftBackDrive, double speedLeftFrontDrive, double speedRightBackDrive, double speedRightFrontDrive){
        double deltaTime = System.currentTimeMillis() - lastTime;
        double deltaSpeedLeftFrontDrive = speedLeftFrontDrive - lastSpeedLeftFrontDrive;
        double deltaSpeedLeftBackDrive = speedLeftBackDrive - lastSpeedLeftBackDrive;
        double deltaSpeedRightFrontDrive = speedRightFrontDrive - lastSpeedRightFrontDrive;
        double deltaSpeedRightBackDrive = speedRightBackDrive - lastSpeedRightBackDrive;
        double maxDelta = deltaTime * accelerationLimit;
        
        double[] maxList = {
            1.0,
            Math.abs(deltaSpeedLeftBackDrive / maxDelta),
            Math.abs(deltaSpeedLeftFrontDrive / maxDelta),
            Math.abs(deltaSpeedRightBackDrive / maxDelta),
            Math.abs(deltaSpeedRightFrontDrive / maxDelta)
            };

        double maxMultiplier = returnMax(maxList);

        leftBackDriveEx.setVelocity(speedLeftBackDrive - deltaSpeedLeftBackDrive + deltaSpeedLeftBackDrive / maxMultiplier);
        leftFrontDriveEx.setVelocity(speedLeftFrontDrive - deltaSpeedLeftFrontDrive + deltaSpeedLeftFrontDrive / maxMultiplier);
        rightBackDriveEx.setVelocity(speedRightBackDrive - deltaSpeedRightBackDrive + deltaSpeedRightBackDrive / maxMultiplier);
        rightFrontDriveEx.setVelocity(speedRightFrontDrive - deltaSpeedRightFrontDrive + deltaSpeedRightFrontDrive / maxMultiplier);

        lastTime = System.currentTimeMillis();
        lastSpeedLeftFrontDrive = speedLeftFrontDrive;
        lastSpeedLeftBackDrive = speedLeftBackDrive;
        lastSpeedRightBackDrive = speedRightBackDrive;
        lastSpeedRightFrontDrive = speedRightFrontDrive;

        telemetry.addData("maxMultiplier", maxMultiplier);
    }

    public double returnMax(double[] intList){
        double maxInt=0;
        for (double INT : intList) {
            maxInt=Math.max(maxInt, INT);
        }
        return maxInt;
    }
}