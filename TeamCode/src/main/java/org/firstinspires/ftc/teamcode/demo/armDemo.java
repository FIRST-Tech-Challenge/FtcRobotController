package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class armDemo extends LinearOpMode {

    private DcMotor susan;
    private DcMotor thera;

    @Override
    public void runOpMode() {
        susan = hardwareMap.get(DcMotor.class, "carousel");
        thera = hardwareMap.get(DcMotor.class, "binderClip");

        susan.setDirection(DcMotor.Direction.FORWARD);
        thera.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Initialized", "Ready to start");
        waitForStart();

        susan.setPower(1);
//        thera.setPower(0.5);
        sleep(5000);
        susan.setPower(0);
//        thera.setPower(0);
    }
}
