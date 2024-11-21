package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;

import java.util.List;

/** config
 * deposit Left Arm Position initial position for installation = 0
 * deposit Left Arm Position initial position = 0
 *
 */

@Config
@TeleOp(name = "TeleOps_Mecanum_FMS", group = "OpMode")
public class BasicTeleOps extends OpMode {
    //Robot
    public RobotHardware robot;                     // Bring in robot hardware configuration

    public GamepadEx gamepadCo1;                    //For gamepad
    //public GamepadEx gamepadCo2;

    //Robot drive
    public RobotDrive robotDrive;                   //For robot drive

    //Robot Intake & Deposit
    public FiniteMachineStateArm depositArmDrive;   //For Robot Arm

    //public Color_sensor colorSensor;

    //For ServoTest
    public ServoTest depositServoTest;              //For Deposit Intake Servo testing

    private FtcDashboard dashboard;                 //For Dashboard
    private TelemetryManager telemetryManager;      //For Telemetry

    //Bulk Reading
    private List<LynxModule> allHubs;

    //Drive power factor
    public static double powerFactor = 0.5;

    //Intake Parameter Configure
    public static double intake_Idle = 0.3;
    public static double intake_Dump = 0.0;

    public static double intake_Slide_Initial   = 0.4;
    public static double intake_slide_Extension = 0.65;
    public static double intake_slide_Retract   = 0.4;

    public static double intake_Rotation        = 0.4;

    public static double intake_Arm_initial     = 0.4;
    public static double intake_Arm_down        = 0.1;
    public static double intake_Arm_retract     = 0.55;

    public static double intake_Claw_Open       = 0.1;
    public static double intake_Claw_Close      = 0.2;

    //Deposit Config
    public static double deposit_Slide_down_Pos         = 50;   //slides Position Configure
    public static double deposit_Slide_Highbar_Pos      = 795;  //slides Position Configure
    public static double deposit_Slide_Highbasket_Pos   = 3000; //slides Position Configure

    public static double deposit_Wrist_dump_Pos         = 0.3;
    public static double deposit_Wrist_retract_Pos      = 0;

    public static double deposit_Arm_dump_Pos           = 0.8;
    public static double deposit_Arm_retract_Pos        = 0.05;

    public static double deposit_Arm_hook_Pos           = 0.8;
    public static double deposit_Wrist_hook_Pos         = 0.3;

    public static double dumpTime                       = 1.5;
    public static double retractTime                    = 2.8;

    public static double deposit_Slide_UpLiftPower      = 0.7;  //slides power
    public static double downLiftPower                  = 0.5;  //slides power

    
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        telemetryManager = new TelemetryManager(telemetry);

        robot = new RobotHardware();
        robot.init(hardwareMap);                                                        // Initialize hardware in RobotHardware

        gamepadCo1 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, telemetryManager,powerFactor);   // Pass robot instance to RobotDrive
        robotDrive.Init();                                                              // Initialize RobotDrive

        /**
        //vertical slide arm control
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetryManager, dump_Idle, dump_Deposit, dropTime, retractTime, intake_Idle, intake_Dump, downLiftpos, upLiftmid,upLiftpos, upLiftPower, downLiftPower); // Pass parameters as needed);
        depositArmDrive.Init();
        **/

        //
        depositServoTest = new ServoTest(robot,gamepadCo1,telemetryManager);
        depositServoTest.ServoTestInit();

        //colorSensor = new Color_sensor(robot);                                          // Initialize color sensor

        // get bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //
        telemetry.addLine("-------------------");
        telemetryManager.update("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
    }

    @Override
    public void loop() {
        /**
        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                // Example: Reading motor position for each hub
                if (hub.equals(allHubs.get(0))) { // Assuming the first hub is Control Hub
                    int frontLeftMotor = bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber());
                    int frontRightMotor = bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber());

                    telemetryManager.update("Deposit Left Motor Position", frontLeftMotor);
                    telemetryManager.update("Deposit right Motor Position", frontRightMotor);
                } else if (hub.equals(allHubs.get(1))) { // Assuming the second hub is Expansion Hub
                    int liftMotorLeft = bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber());
                    int liftMotorRight = bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber());

                    telemetryManager.update("Motor (Expansion Hub) Position", liftMotorLeft);
                    telemetryManager.update("Analog Sensor (Expansion Hub)", liftMotorRight);
                }
            }

        }
         **/

        robotDrive.DriveLoop(); // Use RobotDrive methods
        //depositArmDrive.VsArmLoop();

        //depositArmTest
        depositServoTest.ServoTestLoop();


        // Real-time telemetry data to Driver Station
        //telemetryManager.update("Front Left Motor Power", robot.frontLeftMotor.getPower());
        //telemetryManager.update("Front Right Motor Power", robot.frontRightMotor.getPower());
        //telemetryManager.update("Back Left Motor Power", robot.backLeftMotor.getPower());
        //telemetryManager.update("Back Right Motor Power", robot.backRightMotor.getPower());
        //telemetryManager.update("Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        //telemetryManager.update("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
        //telemetryManager.update("Color Sensor red", colorSensor.getColor()[0]);
        //telemetryManager.update("Color Sensor green", colorSensor.getColor()[1]);
        //telemetryManager.update("Color Sensor blue", colorSensor.getColor()[2]);
        //telemetryManager.update("Lift State", depositArmDrive.State().toString());
        //telemetryManager.update("Servo Intake position", robot.IntakeServo.getPosition());
        //telemetryManager.update("Servo Intake Arm position", robot.IntakeArmServo.getPosition());

        telemetry.addData("deposit Left Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("deposit Right Arm Position", robot.depositRightArmServo.getPosition());
        telemetry.addData("deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("lift slide motor position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("right slide motor position", robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        //robot.IntakeServo.setPosition(1.0);
        telemetryManager.update("Status", "Robot stopped");
    }
}
