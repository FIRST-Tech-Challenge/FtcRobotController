package org.firstinspires.ftc.teamcode.TeleOps.Mechanisms.PowerPlayMechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOps.Mechanisms.PID;


public class SyntaxErrorMechanisms {

    final private DcMotor[] motors; //motors[0] = left //motors[1] = right
    final private CRServo[] servos;

    PID pid = new PID(0.008, 0, 0);

    public SyntaxErrorMechanisms(DcMotor[] motors, CRServo[] servos) {
        this.motors = motors;
        this.servos = servos;
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void runManual(double dr4bp, double ip) {
        motors[0].setPower(-dr4bp * 0.75 - 0.11);
        motors[1].setPower(dr4bp * 0.75 + 0.11);
        for (CRServo servo : servos) {
            servo.setPower(ip);
        }
    }
    public void runIntake(double ip) {
        for (CRServo servo :servos) {
            servo.setPower(ip);
        }
    }
    public void runPID(double target) {
        double command = pid.update(-motors[0].getCurrentPosition(), target) + 0.11;
        motors[0].setPower(-command);
        motors[1].setPower(command);
    }
    public void reset() {

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public double getPosition() {
        return motors[1].getCurrentPosition();
    }
}
