package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.commands.ArmDriveBackward;
import org.firstinspires.ftc.teamcode.commands.ArmDriveForward;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveBackward;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveForward;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


@TeleOp(name="RobotTeleop", group="Competition")
public class RobotTeleop extends CommandOpMode {

    static final String DRIVE_MODE = "RC";
    static final Boolean INTAKE_ENABLED = true;
    static final Boolean ARM_ENABLED = true;
    static final Boolean CAROUSEL_ENABLED = true;


    @Override
    public void initialize() {

        telemetry.setAutoClear(true);

        // Drive Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

        Sensors m_sensors = new Sensors(colorSensor, touchSensor, telemetry);




        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.reset();

        BNO055IMU m_imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);
        GamepadEx m_operatorGamepad = new GamepadEx(gamepad2);



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
            MotorEx motorIntakeLeft = new MotorEx(hardwareMap, "motorIntakeLeft");
            MotorEx motorIntakeRight = new MotorEx(hardwareMap, "motorIntakeRight");

            Intake m_intake = new Intake(motorIntakeLeft, motorIntakeRight, telemetry);

            GamepadButton driver_a = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.A);
            GamepadButton driver_y = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.Y);

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

            MotorEx motorArm = new MotorEx(hardwareMap, "motorArm", Motor.GoBILDA.RPM_312);
            motorArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorArm.resetEncoder();

            Arm m_arm = new Arm(motorArm, telemetry);

            GamepadButton oper_dpad_left = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            GamepadButton oper_dpad_right = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

            oper_dpad_left.whileHeld(new ArmDriveBackward(m_arm, telemetry)).whenReleased(() -> m_arm.stopAll());
            oper_dpad_right.whileHeld(new ArmDriveForward(m_arm, telemetry)).whenReleased(() -> m_arm.stopAll());
        }

        if (CAROUSEL_ENABLED) {
            /* Carousel subsystem

            Enabled/disabled at the top setting the ARM_ENABLED value to true/false


            Holding the DPAD_LEFT drives the arm backwards at a set speed using the CarouselDriveBackward
            Holding the DPAD_RIGHT drives the arm forward at a set speed using the CarouselDriveForward
             */

            MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

            Carousel m_carousel = new Carousel(motorCarousel, telemetry);

            GamepadButton oper_dpad_up = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            GamepadButton oper_dpad_down = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

            oper_dpad_up.whileHeld(new CarouselDriveBackward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll());
            oper_dpad_down.whileHeld(new CarouselDriveForward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll());
        }

        telemetry.addLine("Robot Initialized");

    }
}
