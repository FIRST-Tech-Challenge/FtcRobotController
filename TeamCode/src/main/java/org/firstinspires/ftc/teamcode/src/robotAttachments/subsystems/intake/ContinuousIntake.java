package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public class ContinuousIntake implements Controllable {
    /**
     * The power for going forward for the front motor
     */
    private final static double frontMotorForwardPower = 1;

    /**
     * The power for going forward for the back motor
     */
    private final static double backMotorForwardPower = 1;

    /**
     * Front DcMotor Object
     */
    private final DcMotor frontIntakeMotor;

    /**
     * Back DcMotor Object
     */
    private final DcMotor backIntakeMotor;

    public ContinuousIntake(HardwareMap hardwareMap, String frontMotorName,String backMotorName) {
        frontIntakeMotor = hardwareMap.dcMotor.get(frontMotorName);
        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backIntakeMotor = hardwareMap.dcMotor.get(backMotorName);
        backIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * this turns on the intake motor to intake freight
     */
    public void turnIntakeOn() {
        frontIntakeMotor.setPower(frontMotorForwardPower);
        backIntakeMotor.setPower(backMotorForwardPower);
    }

    /**
     * turns off the intake motor
     */
    public void turnIntakeOff() {
        frontIntakeMotor.setPower(0);
        backIntakeMotor.setPower(0);
    }

    /**
     * reverses the intake motor to remove freight from the intake bucket
     */
    public void turnIntakeReverse() {
        frontIntakeMotor.setPower(-frontMotorForwardPower);
        backIntakeMotor.setPower(-backMotorForwardPower);
    }

    /**
     * @param power a variable input for the power of the intake motor
     *              this sets the power of the intake motor to the power variable input
     */
    public void setMotorPower(double power) {
        frontIntakeMotor.setPower(power * frontMotorForwardPower);
        backIntakeMotor.setPower(power * backMotorForwardPower);
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

    public void setFrontMotorPower(double power){
        frontIntakeMotor.setPower(power);
    }

    public void setBackMotorPower(double power){
        backIntakeMotor.setPower(power);
    }

}
