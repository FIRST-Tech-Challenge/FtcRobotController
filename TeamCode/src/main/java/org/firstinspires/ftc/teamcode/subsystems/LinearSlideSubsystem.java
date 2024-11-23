package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSubsystem {
    private DcMotorEx motor;
    private OpMode opMode;
    private HardwareMap hardwareMap;

    public LinearSlideSubsystem(OpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //to check whether it is currently in motion motor.getBusy()
    public void setPosition(int targetPositionTicks) {
        motor.setTargetPosition(targetPositionTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1);
    }

    public void stopMotor() {
        motor.setPower(0);
    }

    public void runPIDPosition(int targetPosition) {
        if (motor.getCurrentPosition() == targetPosition) {
            motor.setPower(0);
            return;
        }
        PIDController controller = new PIDController(0, 0, 0);

        motor.setPower(controller.calculate(targetPosition));

    }
}
