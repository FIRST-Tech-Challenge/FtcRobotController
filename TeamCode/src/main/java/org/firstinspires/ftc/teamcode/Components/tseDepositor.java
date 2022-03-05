package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class tseDepositor {
    // Define class members
    CRServo tseCrServo;
    LinearOpMode op;
    public tseDepositor(LinearOpMode opMode) {
        op = opMode;
        tseCrServo = opMode.hardwareMap.get(CRServo.class, "crtsedepositer");
    }
    public void moveTseDepositerTape(String name, int inch,  int forward) {
        String caption;

        tseCrServo.setPower(0);
        caption = "Servo " + name + " " + forward + " " +inch ;
        op.telemetry.addData(caption, " Moving ");
        op.telemetry.update();


        if (forward != 0) {
            tseCrServo.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            tseCrServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        tseCrServo.setPower(1.0);
        if (forward == 1){
            op.sleep(147*inch);
        } else {
            op.sleep(124 * inch);
        }
        tseCrServo.setPower(0);
    }
    public void setTseCrServoPower(double power) {
        tseCrServo.setPower(-power);
    }
}
