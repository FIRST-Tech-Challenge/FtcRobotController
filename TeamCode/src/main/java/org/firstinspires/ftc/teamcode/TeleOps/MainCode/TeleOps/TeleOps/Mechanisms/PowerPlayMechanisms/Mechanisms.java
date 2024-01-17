package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Mechanisms.PowerPlayMechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Mechanisms.PID;

public class Mechanisms extends SubsystemBase {
    final private DcMotor l;
    final private DcMotor r;
    final private CRServo il;
    final private CRServo ir;
    PID pid = new PID(0.008, 0, 0);

    public Mechanisms(DcMotor lm, DcMotor rm, CRServo iLm, CRServo iRm) {
        l = lm;
        r = rm;
        il = iLm;
        ir = iRm;
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void runIntake(double power) {
        il.setPower(power);
        ir.setPower(-power);
    }

    public void runManual(double fbp) {
        l.setPower(fbp);
        r.setPower(-fbp);
    }
    public void runManual(double fbp, double rp) {
        l.setPower(-fbp);
        r.setPower(fbp);
        il.setPower(rp);
        ir.setPower(rp);
    }
    public void runPID(double target) {
        double command = pid.update(l.getCurrentPosition(), target) + 0.11;
        l.setPower(command);
        r.setPower(-command);
    }

    public double currentPosition(){
        return l.getCurrentPosition();
    }
}