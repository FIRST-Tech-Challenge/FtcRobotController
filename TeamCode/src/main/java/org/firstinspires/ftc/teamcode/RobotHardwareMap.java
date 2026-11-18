package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.vision.VisionPortal;

public class RobotHardwareMap {

//    private LinearOpMode linearOpMode = null;
    private OpMode opMode = null;
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

    public DcMotorEx transfer;
    public DcMotorEx intake;
    public DcMotorEx launcherPollen;
    public DcMotorEx launcherNectar;

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
    private CRServo PollenServo;
    private CRServo NectarServo;
    private CRServo FlowerServo;
    public Servo servo;

//    public RobotHardwareMap(HardwareMap baseHardwareMap, LinearOpMode operationMode) {
//        this.linearOpMode = operationMode;
//        this.baseHardwareMap = baseHardwareMap;
////        this.initialize();
//    }

    public RobotHardwareMap(HardwareMap baseHardwareMap,OpMode operationMode) {
        this.opMode = operationMode;
        this.baseHardwareMap = baseHardwareMap;
//        this.initialize();
    }

//    public RobotHardwareMap(HardwareMap baseHardwareMap, OpMode operationMode) {
//        this.opMode = (LinearOpMode) operationMode;
//        this.baseHardwareMap = baseHardwareMap;
//    }

    public void initialize() {

        opMode.telemetry.addData("Status", "detecting...");

        controlHubBatteryVoltage = baseHardwareMap.get(VoltageSensor.class, "Control Hub");
        controlHub = baseHardwareMap.get(LynxModule.class, "Control Hub");
        expansionHubBatteryVoltage = baseHardwareMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
        expansionHub = baseHardwareMap.tryGet(LynxModule.class, "Expansion Hub 2");


//        testMotor3 = baseHardwareMap.get(DcMotorEx.class, "testMotor3");

//        baseHardwareMap.get(DcMotorEx.class, "testMotor3");

        //Shooter Motors

        transfer = baseHardwareMap.get(DcMotorEx.class, "transfer_motor");
        intake = baseHardwareMap.get(DcMotorEx.class, "intake_motor");
        launcherPollen = baseHardwareMap.get(DcMotorEx.class, "launcher_pollen");
        launcherNectar = baseHardwareMap.get(DcMotorEx.class, "launcher_nectar");

        //Drive Motors
        backLeftMotor = baseHardwareMap.get(DcMotorEx.class, "RL");
        backRightMotor = baseHardwareMap.get(DcMotorEx.class, "RR");
        frontLeftMotor = baseHardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor = baseHardwareMap.get(DcMotorEx.class, "FR");

//        intakeMotorFront = baseHardwareMap.tryGet(DcMotorEx.class, "IF");
//        outtakeMotorBack1 = baseHardwareMap.tryGet(DcMotorEx.class, "BRO");
//        outtakeMotorBack2 = baseHardwareMap.tryGet(DcMotorEx.class, "BLO");
//        clawRotator = baseHardwareMap.tryGet(DcMotorEx.class, "clawRotator");
//        armMotor = baseHardwareMap.tryGet(DcMotorEx.class, "armMotor");

//        LED1Green = baseHardwareMap.tryGet(DigitalChannel.class, "LED1Green");
//        LED1Red = baseHardwareMap.tryGet(DigitalChannel.class, "LED1Red");
//        LED2Green = baseHardwareMap.tryGet(DigitalChannel.class, "LED2Green");
//        LED2Red = baseHardwareMap.tryGet(DigitalChannel.class, "LED2Red");

//        LeftTransferServo = baseHardwareMap.tryGet(CRServo.class, "LTS");
//        RightTransferServo = baseHardwareMap.tryGet(CRServo.class, "RTS");
//        CenterTransferServo = baseHardwareMap.tryGet(CRServo.class, "CTS");

        PollenServo = baseHardwareMap.tryGet(CRServo.class, "pollen_servo");
        NectarServo = baseHardwareMap.tryGet(CRServo.class, "nectar_servo");
        FlowerServo = baseHardwareMap.tryGet(CRServo.class, "flower_servo");
        servo = baseHardwareMap.tryGet(Servo.class,"servo");
        chImu = baseHardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        chImu.initialize(myIMUParameters);

        opMode.telemetry.addData("Status", "done");
        opMode.telemetry.update();
    }


        //    private LinearOpMode linearOpMode = null;
//        private OpMode opMode = null;
//        public HardwareMap baseHMap;

    public boolean hasTransfer() {
        return LeftTransferServo != null && RightTransferServo != null && CenterTransferServo != null;
    }
    public boolean hasDrive(){
        return backLeftMotor != null && backRightMotor != null && frontLeftMotor != null && frontRightMotor != null;
    }
    public boolean hasArm() {
        return armMotor != null && clawRotator != null;
    }

    public CRServo getFlowerServo() {
        return FlowerServo;
    }
    public boolean hasNectarServo(){
        return NectarServo != null;
    }
    public boolean hasPollenServo(){
        return PollenServo != null;
    }
    public boolean hasFlowerServo(){
        return FlowerServo != null;
    }
    public void setFlowerServo(CRServo flowerServo) {
        FlowerServo = flowerServo;
    }

    public CRServo getNectarServo() {
        return NectarServo;
    }

    public void setNectarServo(CRServo nectarServo) {
        NectarServo = nectarServo;
    }

    public CRServo getPollenServo() {
        return PollenServo;
    }

    public void setPollenServo(CRServo pollenServo) {
        PollenServo = pollenServo;
    }
}
