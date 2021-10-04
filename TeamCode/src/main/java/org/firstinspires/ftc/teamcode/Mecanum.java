package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Disabled
@TeleOp(name = "Basic Mecanum Drive")
public class Mecanum extends LinearOpMode {
  
  // private DcMotor armMotor;
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_1620);
    MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_1620);
    MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_1620);
    MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_1620);

    CRServo servoIntakeLeft = new CRServo(hardwareMap, "servoIntakeLeft");
    CRServo servoIntakeRight = new CRServo(hardwareMap, "servoIntakeRight");

    GamepadEx driver = new GamepadEx(gamepad1);

    Intake m_intake = new Intake(servoIntakeLeft, servoIntakeRight, telemetry);

    GamepadButton driver_x = driver.getGamepadButton(GamepadKeys.Button.X);
    GamepadButton driver_b = driver.getGamepadButton(GamepadKeys.Button.B);

    MecanumDrive drive = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
    motorFrontLeft.setInverted(true);
    motorBackLeft.setInverted(true);
    motorFrontRight.setInverted(true);
    motorBackRight.setInverted(true);

    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.

      while (opModeIsActive()) {

        drive.driveRobotCentric(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

        if (gamepad1.b) {
          servoIntakeLeft.set(0.25);
        } else if (gamepad1.x) {
          servoIntakeRight.set(-0.25);
        }

        telemetry.addData("Button X", driver_x.get());
        telemetry.addData("Button B", driver_b.get());
        telemetry.update();

      }
    }
  }
}