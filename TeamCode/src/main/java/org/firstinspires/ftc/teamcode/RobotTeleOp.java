package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.commands.ArmDriveBackward;
import org.firstinspires.ftc.teamcode.commands.ArmDriveForward;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.io.IOException;


@TeleOp(name="Robot TeleOp", group="TeleOp")
public class RobotTeleOp extends CommandOpMode {

    Utils utils = new Utils();
    static final String DRIVE_MODE = "RC";


    @Override
    public void initialize() {

        telemetry.setAutoClear(false);

        // Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_223);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_223);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_223);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_223);
        

//        motorBackRight.setInverted(true);
//        motorFrontRight.setInverted(true);
//        motorBackLeft.setInverted(true);
//        motorFrontLeft.setInverted(true);

//        MotorEx motorArm = new MotorEx(hardwareMap, "motorArm");
//        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

//        CRServo servoIntakeLeft = new CRServo(hardwareMap, "servoIntakeLeft");
//        CRServo servoIntakeRight = new CRServo(hardwareMap, "servoIntakeRight");

        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
//        m_gyro.reset();

        BNO055IMU m_imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);


        // Subsystems
        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight,
                telemetry, m_gyro, DRIVE_MODE, m_imu);

//        Arm m_arm = new Arm(motorArm, telemetry);
//        Intake m_intake = new Intake(servoIntakeLeft, servoIntakeRight, telemetry);

        Vision m_vision = new Vision(hardwareMap, telemetry);
        register(m_vision);


        //Buttons
        GamepadButton dpad_up = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton dpad_down = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        GamepadButton dpad_left = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        GamepadButton dpad_right = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);


        GamepadButton driver_a = m_driverGamepad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton driver_y = m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y);


//        dpad_up.whenPressed(new ArmToPosition(m_arm, 100));
//        dpad_down.whenPressed(new ArmToPosition(m_arm, -100));
//        dpad_left.whileHeld(new ArmDriveBackward(m_arm));
//        dpad_right.whileHeld(new ArmDriveForward(m_arm));



//        driver_a.whileHeld(new IntakeIn(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());
//        driver_y.whileHeld(new IntakeOut(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());


//        // Default Command
        register(m_defaultdrive);
        m_defaultdrive.setDefaultCommand(new DefaultDrive(m_defaultdrive, m_driverGamepad, telemetry));

        telemetry.addLine("Robot Initialized");

    }
}
