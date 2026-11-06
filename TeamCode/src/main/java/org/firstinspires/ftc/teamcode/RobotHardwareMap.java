package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.vision.VisionPortal;

public class RobotHardwareMap {

    private LinearOpMode opMode = null;
    public HardwareMap baseHardwareMap;

    public VoltageSensor controlHubBatteryVoltage;
    public VoltageSensor expansionHubBatteryVoltage;

    public LynxModule controlHub;
    public LynxModule expansionHub;
    public DcMotorEx testMotor3;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx intakeMotorFront;
    public DcMotorEx outtakeMotorBack1;
    public DcMotorEx outtakeMotorBack2;
    public DcMotorEx clawRotator;
    public DcMotorEx armMotor;

    public DigitalChannel LED1Green;
    public DigitalChannel LED1Red;
    public DigitalChannel LED2Green;
    public DigitalChannel LED2Red;

    public CRServo LeftTransferServo;
    public CRServo RightTransferServo;
    public CRServo CenterTransferServo;

    public IMU chImu;

    private final int baseResolution_x = 320;
    private final int baseResolution_y = 240;

    private VisionPortal visionPortal;

    boolean controlHubBatteryVoltageEnabled = true;
    boolean expansionHubBatteryVoltageEnabled = true;

    public RobotHardwareMap(HardwareMap baseHardwareMap, LinearOpMode opmode) {
        this.opMode = opmode;
        this.baseHardwareMap = baseHardwareMap;
    }

    public void initialize() {

        opMode.telemetry.addData("Status", "detecting...");

        controlHubBatteryVoltage = baseHardwareMap.get(VoltageSensor.class, "Control Hub");
        controlHub = baseHardwareMap.get(LynxModule.class, "Control Hub");
        expansionHubBatteryVoltage = baseHardwareMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
        expansionHub = baseHardwareMap.tryGet(LynxModule.class, "Expansion Hub 2");
        testMotor3 = baseHardwareMap.get(DcMotorEx.class, "testMotor3");
        backLeftMotor = baseHardwareMap.get(DcMotorEx.class, "RL");
        backRightMotor = baseHardwareMap.get(DcMotorEx.class, "RR");
        frontLeftMotor = baseHardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor = baseHardwareMap.get(DcMotorEx.class, "FR");

        //intakeMotorFront = baseHMap.tryGet(DcMotorEx.class, "IF");
        outtakeMotorBack1 = baseHardwareMap.tryGet(DcMotorEx.class, "BRO");
        outtakeMotorBack2 = baseHardwareMap.tryGet(DcMotorEx.class, "BLO");
        clawRotator = baseHardwareMap.tryGet(DcMotorEx.class, "clawRotator");
        armMotor = baseHardwareMap.tryGet(DcMotorEx.class, "armMotor");

        LED1Green = baseHardwareMap.tryGet(DigitalChannel.class, "LED1Green");
        LED1Red = baseHardwareMap.tryGet(DigitalChannel.class, "LED1Red");
        LED2Green = baseHardwareMap.tryGet(DigitalChannel.class, "LED2Green");
        LED2Red = baseHardwareMap.tryGet(DigitalChannel.class, "LED2Red");

        LeftTransferServo = baseHardwareMap.tryGet(CRServo.class, "LTS");
        RightTransferServo = baseHardwareMap.tryGet(CRServo.class, "RTS");
        CenterTransferServo = baseHardwareMap.tryGet(CRServo.class, "CTS");

        chImu = baseHardwareMap.get(IMU.class, "chImu");

        IMU.Parameters myIMUParamaters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        chImu.initialize(myIMUParamaters);

        opMode.telemetry.addData("Status", "done");
        opMode.telemetry.update();
    }

    public boolean hasIntake() {
        return intakeMotorFront != null;
    }

    public boolean hasOuttake() {
        return outtakeMotorBack1 != null && outtakeMotorBack2 != null;
    }

    public boolean hasTransfer() {
        return LeftTransferServo != null && RightTransferServo != null && CenterTransferServo != null;
    }

    public boolean hasArm() {
        return armMotor != null && clawRotator != null;
    }

    public boolean hasLEDs() {
        return LED1Green != null && LED1Red != null && LED2Green != null && LED2Red != null;
    }
}