package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp
public class hello extends LinearOpMode {
    private DcMotor m1, m2, m3, m4;
    private double speed;
//    private DcMotor[] motor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m1 = hardwareMap.dcMotor.get("m1");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");
        DcMotor m3 = hardwareMap.dcMotor.get("m3");
        DcMotor m4 = hardwareMap.dcMotor.get("m4");

        Gamepad p1 = new Gamepad();

//        motor = type DcNotor[] {
//            hardwareMap.dcMotor.get("m1"),
//            hardwareMap.dcMotor.get("m2"),
//            hardwareMap.dcMotor.get("m3"),
//            hardwareMap.dcMotor.get("m4")
//        };

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
                speed = gamepad1.left_stick_y;
                m1.setPower(speed);
                m2.setPower(speed);
                m3.setPower(-speed);
                m4.setPower(-speed);
        }
    }
}

