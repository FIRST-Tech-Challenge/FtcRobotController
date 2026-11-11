package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Intake {

    RobotHardwareMap robotHardwareMap;
    OpMode opMode;
    private double speed;
    private char direction;

    // finish the init method

    public Intake(RobotHardwareMap robotHardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
    }


    public void initialize() {
        robotHardwareMap.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardwareMap.intake.setDirection(DcMotor.Direction.REVERSE);
        robotHardwareMap.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setSpeed(double speed_in) {
        speed = speed_in;
        robotHardwareMap.intake.setPower(speed);

    }

    public void Toggle() {
        if (speed > 0) {
            speed = 0;
        } else {
            speed = 1;
        }

        robotHardwareMap.intakeMotorFront.setPower(speed);
        robotHardwareMap.intakeMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void on() {
        speed = 1;
        robotHardwareMap.intakeMotorFront.setPower(speed);
        robotHardwareMap.intakeMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void off() {
        speed = 0;
        robotHardwareMap.intakeMotorFront.setPower(speed);
        robotHardwareMap.intakeMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}