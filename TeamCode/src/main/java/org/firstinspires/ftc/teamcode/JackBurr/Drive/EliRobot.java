package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EliRobot extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public Servo leftIntakeWheel;
    public Servo rightIntakeWheel;
    public ElapsedTime buttonTimer = new ElapsedTime();

    public enum IntakeState {
        INWARD,
        OUTWARD
    }
    RobotV2Config config = new RobotV2Config();
    public IntakeState intakeState;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
        rightIntakeWheel = hardwareMap.get(Servo.class, "rightIntakeWheel");
        leftIntakeWheel = hardwareMap.get(Servo.class, "leftIntakeWheel");
        intakeState = IntakeState.INWARD;
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        drive(y, x, rx);
        runIntake();
        if (gamepad1.x && buttonTimer.seconds() > 0.3 && intakeState == IntakeState.OUTWARD){
            intakeState = IntakeState.INWARD;
        }
        else if (gamepad1.x && buttonTimer.seconds() > 0.3 && intakeState == IntakeState.INWARD){
            intakeState = IntakeState.OUTWARD;
        }
    }

    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void runIntake(){
        switch (intakeState){
            case INWARD:
                leftIntakeWheel.setPosition(0);
                rightIntakeWheel.setPosition(1);
            case OUTWARD:
                leftIntakeWheel.setPosition(1);
                rightIntakeWheel.setPosition(0);
        }
    }

}
