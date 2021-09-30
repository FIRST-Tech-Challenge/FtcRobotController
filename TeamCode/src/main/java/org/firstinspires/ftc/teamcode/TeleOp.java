package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class TeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        // Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_1620);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_1620);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_1620);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_1620);

        motorBackRight.setInverted(true);
        motorFrontRight.setInverted(true);

//        MotorEx motorArm = new MotorEx(hardwareMap, "motorArm");
//        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");
//
//        CRServo servoIntakeLeft = new CRServo(hardwareMap, "servoIntakeLeft");
//        CRServo servoIntakeRight = new CRServo(hardwareMap, "servoIntakeRight");

        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.reset();

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);


        // Subsystems
        //Subsystems
        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight,
                telemetry, m_gyro, "RC");

        m_defaultdrive.setDefaultCommand(new DefaultDrive(m_defaultdrive, m_driverGamepad));

    }


}
