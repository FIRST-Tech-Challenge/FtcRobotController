package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="testing", group = "Real")
public class motorTest extends OpMode {

    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    public DcMotorEx extenderArm;
    public CRServo intakeServo;
    public Servo wristServo;

    @Override
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LEFT_ARM_MOTOR_NAME);
        motorRight = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.RIGHT_ARM_MOTOR_NAME);
        extenderArm = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);

        intakeServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);
        wrist

        motorLeft.setDirection(Constants.IntakeConstants.LEFT_ARM_MOTOR_DIRECTION);
        motorRight.setDirection(Constants.IntakeConstants.RIGHT_ARM_MOTOR_DIRECTION);
        extenderArm.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);
    }

    @Override
    public void loop() {
        double elbowArm = -gamepad1.left_stick_y;

        motorLeft.setPower(elbowArm);
        motorRight.setPower(elbowArm);
        extenderArm.setPower(gamepad1.right_trigger);

        intakeServo.setPower(gamepad1.left_trigger);
        wristServo.rotateBy(gamepad1.y ? 50 : 0);

        telemetry.addData("Elbow Motor Power", "%.2f", elbowArm);
        telemetry.addData("Extender Arm Power", "%.2f", gamepad1.right_trigger);
        telemetry.update();
    }
}
