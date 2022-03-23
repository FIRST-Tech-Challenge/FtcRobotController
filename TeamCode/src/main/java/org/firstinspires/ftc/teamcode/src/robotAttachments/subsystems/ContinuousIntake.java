package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public class ContinuousIntake implements Controllable {
    /**
     * The power for going forward
     */
    private final static double forwardPower = 1;

    /**
     * DcMotor Object
     */
    private final DcMotor intakeMotor;

    public ContinuousIntake(HardwareMap hardwareMap, String motorName) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * this turns on the intake motor to intake freight
     */
    public void setIntakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    /**
     * turns off the intake motor
     */
    public void setIntakeOff() {
        intakeMotor.setPower(0);
    }

    /**
     * reverses the intake motor to remove freight from the intake bucket
     */
    public void setIntakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    /**
     * @param power a variable input for the power of the intake motor
     *              this sets the power of the intake motor to the power variable input
     */
    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    @Override
    public Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2) {

        if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {

            this.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
        } else {
            this.setMotorPower(0);
        }
        return null;
    }
}
