package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake2020 implements EbotsManip2020 {
    private DcMotorEx intakeMotor;

    public Intake2020(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Deprecated
    public void setPower(double targetPower){
        this.intakeMotor.setPower(targetPower);
    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        double inputThreshold = 0.3;
        double intakeInput = -gamepad.left_stick_y;

        // Condition the input signal to either be -1, 0, or 1
        double intakePower = (Math.abs(intakeInput) < inputThreshold) ? 0 : Math.signum(intakeInput) * 1;

        intakeMotor.setPower(intakePower);
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
    }
}
