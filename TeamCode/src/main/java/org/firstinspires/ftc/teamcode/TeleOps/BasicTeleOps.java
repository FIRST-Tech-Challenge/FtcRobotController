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
@TeleOp(name = "TeleOps_MW_FMS_v1.0", group = "Meet_1")
public class BasicTeleOps extends OpMode {
    //Robot
    public RobotHardware robot;                     // Bring in robot hardware configuration

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    //Robot drive
    public RobotDrive robotDrive;                   //For robot drive

    //Robot Intake & Deposit
    public FiniteMachineStateArm depositArmDrive;   //For Robot Arm
    public FiniteMachineStateIntake intakeArmDrive; //For Robot Intake


    //Bulk Reading
    private List<LynxModule> allHubs;

    //Drive power factor

    //Deposit Config
    public static int deposit_Slide_down_Pos         = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Pos      = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos   = 2800; //slides Position Configure

    public static double deposit_Wrist_dump_Pos         = 0.3;
    public static double deposit_Wrist_retract_Pos      = 0.06;

    public static double deposit_Arm_dump_Pos           = 0.8;
    public static double deposit_Arm_retract_Pos        = 0.0;

    public static double deposit_Arm_hook_Pos           = 0.8;
    public static double deposit_Claw_Open              = 0.11;
    public static double deposit_Claw_Close             = 0.0;

    public static double dumpTime                       = 1.8;
    public static double retractTime                    = 3.2;

    public static int deposit_Slide_UpLiftPower         = 1;  //slides power
    public static double downLiftPower                  = 0.7;  //slides power

    
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware in RobotHardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        //robot configuration

        //gamepad
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //robotDrive
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);   // Pass robot instance to RobotDrive
        robotDrive.Init();                                                              // Initialize RobotDrive


        //Deposit Arm control
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1,gamepadCo2,
                deposit_Arm_retract_Pos, deposit_Arm_dump_Pos,
                dumpTime, retractTime,
                deposit_Wrist_retract_Pos, deposit_Wrist_dump_Pos, deposit_Claw_Open, deposit_Claw_Close,
                deposit_Slide_down_Pos,deposit_Slide_Highbasket_Pos,
                deposit_Slide_UpLiftPower, downLiftPower); // Pass parameters as needed);
        depositArmDrive.Init();

        //Intake Arm Control
        intakeArmDrive = new FiniteMachineStateIntake(robot, gamepadCo1,gamepadCo2);
        intakeArmDrive.Init();

        // get bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //Robot Control State
        RobotDrive.ControlMode currentMode = robotDrive.getControlMode();

        //
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addData("Control Mode", currentMode.name());
        telemetry.addLine("-------------------");
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                // Example: Reading motor position for each hub
                if (hub.equals(allHubs.get(0))) { // Assuming the first hub is Control Hub
                    int liftLeftMotor = bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber());
                    int liftRightMotor = bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber());

                    telemetry.addData("Deposit Left Motor Position (Expansion Hub)", liftLeftMotor);
                    telemetry.addData("Deposit right Motor Position (Expansion Hub)", liftRightMotor);
                } else if (hub.equals(allHubs.get(1))) { // Assuming the second hub is Expansion Hub
                    int frontLeftMotor = bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber());
                    int frontRightMotor = bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber());
                    telemetry.addData("Drive Motor FL Motor (Control Hub) Position", frontLeftMotor);
                    telemetry.addData("Drive Motor FR Motor (Control Hub) Position", frontRightMotor);
                }
            }
        }

        robotDrive.DriveLoop(); // Use RobotDrive methods
        RobotDrive.ControlMode currentMode = robotDrive.getControlMode();

        try {
            depositArmDrive.DepositArmLoop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        FiniteMachineStateArm.LIFTSTATE liftState = depositArmDrive.State();

        intakeArmDrive.IntakeArmLoop();
        FiniteMachineStateIntake.INTAKESTATE intakeState = intakeArmDrive.intakeState();

        // Telemetry
        telemetry.addData("deposit Left Arm Position", robot.depositArmServo.getPosition());
        telemetry.addData("deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Control Mode", currentMode.name());
        telemetry.addData("Heading ", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Lift Mode", liftState.name());
        telemetry.addData("Intake State", intakeState.name());
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
        telemetry.addData("Status", "Robot stopped");
    }
}
