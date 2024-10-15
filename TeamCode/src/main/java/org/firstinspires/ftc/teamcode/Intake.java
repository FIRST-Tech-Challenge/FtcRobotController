package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private CRServo Intake = null;
    private Servo graber1 = null;
    private Servo graber2 = null;
    private LinearOpMode opmode = null;

    public Intake() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        Intake = hwMap.crservo.get("Intake");
        graber1 = hwMap.servo.get("graber1");
        graber2 = hwMap.servo.get("graber2");

        Intake.setPower(0);

    }

    public void intake() {

        Intake.setPower(1);
    }

    public void eject() {
        Intake.setPower(-1);
    }

    public void transport(double speed) {
        Intake.setPower(0);
    }
}
