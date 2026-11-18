    package org.firstinspires.ftc.teamcode;

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
        public HardwareMap baseHMap;

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
        public Servo servo;

        public IMU chImu;

        private final int baseResolution_x = 320;
        private final int baseResolution_y = 240;

        private VisionPortal visionPortal;

        boolean controlHubBatteryVoltageEnabled = true;
        boolean expansionHubBatteryVoltageEnabled = true;

    //    public RobotHardwareMap(HardwareMap baseHardwareMap, LinearOpMode operationMode) {
    //        this.linearOpMode = operationMode;
    //        this.baseHardwareMap = baseHardwareMap;
        ////        this.initialize();
    //    }

        public RobotHardwareMap(HardwareMap baseHMap, OpMode operationMode) {
            this.opMode = operationMode;
            this.baseHMap = baseHMap;
    //        this.initialize();
        }

    //    public RobotHardwareMap(HardwareMap baseHardwareMap, OpMode operationMode) {
    //        this.opMode = (LinearOpMode) operationMode;
    //        this.baseHardwareMap = baseHardwareMap;
    //    }

        public void initialize() {

            opMode.telemetry.addData("Status", "detecting...");

            controlHubBatteryVoltage = baseHMap.get(VoltageSensor.class, "Control Hub");
            controlHub = baseHMap.get(LynxModule.class, "Control Hub");
            expansionHubBatteryVoltage = baseHMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
            expansionHub = baseHMap.tryGet(LynxModule.class, "Expansion Hub 2");


    //        testMotor3 = baseHardwareMap.get(DcMotorEx.class, "testMotor3");

    //        baseHardwareMap.get(DcMotorEx.class, "testMotor3");

            //Shooter Motors

            //transfer = baseHMap.get(DcMotorEx.class, "transfer_motor");
            //intake = baseHMap.get(DcMotorEx.class, "intake_motor");
            //launcherPollen = baseHMap.get(DcMotorEx.class, "launcher_pollen");
            //launcherNectar = baseHMap.get(DcMotorEx.class, "launcher_nectar");

            //Drive Motors
            backLeftMotor = baseHMap.get(DcMotorEx.class, "RL");
            backRightMotor = baseHMap.get(DcMotorEx.class, "RR");
            frontLeftMotor = baseHMap.get(DcMotorEx.class, "FL");
            frontRightMotor = baseHMap.get(DcMotorEx.class, "FR");

            servo = baseHMap.get(Servo.class, "servo");

    //        intakeMotorFront = baseHardwareMap.tryGet(DcMotorEx.class, "IF");
    //        outtakeMotorBack1 = baseHardwareMap.tryGet(DcMotorEx.class, "BRO");
    //        outtakeMotorBack2 = baseHardwareMap.tryGet(DcMotorEx.class, "BLO");

            //LeftTransferServo = baseHMap.tryGet(CRServo.class, "LTS");
            //RightTransferServo = baseHMap.tryGet(CRServo.class, "RTS");
            //CenterTransferServo = baseHMap.tryGet(CRServo.class, "CTS");

            chImu = baseHMap.get(IMU.class, "chImu");

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

        public boolean hasIntake() {
            return intakeMotorFront != null;
        }

        //public boolean hasOuttake() {
            //return outtakeMotorBack1 != null && outtakeMotorBack2 != null;
        }

        //public boolean hasTransfer() {
        //    return LeftTransferServo != null && RightTransferServo != null && CenterTransferServo != null;
        //}

    //    public boolean hasArm() {
            //return armMotor != null && clawRotator != null;
        //}
    //}