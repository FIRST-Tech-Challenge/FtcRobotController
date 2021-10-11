package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.commands.ArmDriveBackward;
import org.firstinspires.ftc.teamcode.commands.ArmDriveForward;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


@TeleOp(name="Robot TeleOp - No Servos", group="Competition")
public class RobotTeleOpNoServos extends CommandOpMode {

    static final String DRIVE_MODE = "RC";
    static final Boolean INTAKE_ENABLED = false;
    static final Boolean ARM_ENABLED = true;

    @Override
    public void initialize() {

        telemetry.setAutoClear(true);

        // Drive Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_223);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_223);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_223);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_223);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

        Sensors m_sensors = new Sensors(colorSensor, touchSensor, telemetry);




        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.reset();

        BNO055IMU m_imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);


        // Drivetrain Subsystem
        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight,
                                                                 motorFrontLeft, motorFrontRight,
                                                                 telemetry, m_gyro, DRIVE_MODE, m_imu);


        /* Default Drive Command
           Set the default command for the drivetrain to be gamepad controlled
        */

        register(m_defaultdrive);
        m_defaultdrive.setDefaultCommand(new DefaultDrive(m_defaultdrive, m_driverGamepad, telemetry));

        Vision m_vision = new Vision(hardwareMap, telemetry);
        register(m_vision);


        if (INTAKE_ENABLED) {
            /* Intake subsystem

            Enabled/disabled at the top setting the INTAKE_ENABLED value to true/false

            Left and right servos are set to continuous rotation.

            Holding the Y button runs them "forward" (out) using the IntakeOut command
            Holding the A button runs them "inward" using the IntakeIn command
            Letting go of the button stops the servos
             */
            CRServo servoIntakeLeft = new CRServo(hardwareMap, "servoIntakeLeft");
            CRServo servoIntakeRight = new CRServo(hardwareMap, "servoIntakeRight");

            Intake m_intake = new Intake(servoIntakeLeft, servoIntakeRight, telemetry);

            GamepadButton driver_a = m_driverGamepad.getGamepadButton(GamepadKeys.Button.A);
            GamepadButton driver_y = m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y);

            driver_a.whileHeld(new IntakeIn(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());
            driver_y.whileHeld(new IntakeOut(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());

        }

        if (ARM_ENABLED) {
            /* Arm subsystem

            Enabled/disabled at the top setting the ARM_ENABLED value to true/false

            Left and right servos are set to continuous rotation.

            Holding the DPAD_UP drives the arm to the setpoint on the encoder using a PID
            Holding the DPAD_DOWN drives the arm to the setpoint on the encoder using a PID
            Holding the DPAD_LEFT drives the arm backwards at a set speed using the ArmDriveBackward
            Holding the DPAD_RIGHT drives the arm forward at a set speed using the ArmDriveForward
             */

            MotorEx motorArm = new MotorEx(hardwareMap, "motorArm");
            motorArm.resetEncoder();

            Arm m_arm = new Arm(motorArm, telemetry);

            GamepadButton dpad_up = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            GamepadButton dpad_down = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            GamepadButton dpad_left = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            GamepadButton dpad_right = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

            dpad_up.whenPressed(new ArmToPosition(m_arm, 100, telemetry));
            dpad_down.whenPressed(new ArmToPosition(m_arm, 0, telemetry));
            dpad_left.whileHeld(new ArmDriveBackward(m_arm, telemetry)).whenReleased(() -> m_arm.stopAll());
            dpad_right.whileHeld(new ArmDriveForward(m_arm, telemetry)).whenReleased(() -> m_arm.stopAll());
        }

        telemetry.addLine("Robot Initialized");

    }
}
