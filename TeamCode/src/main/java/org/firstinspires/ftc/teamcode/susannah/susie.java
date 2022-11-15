package org.firstinspires.ftc.teamcode.susannah;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class susie extends LinearOpMode {
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
    private DcMotor susan;
    private DcMotor thera;

    @Override
    public void runOpMode() {
//        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
//        frontRight = hardwareMap.get(DcMotor.class, "front_right");
//        backLeft = hardwareMap.get(DcMotor.class, "back_left");
//        backRight = hardwareMap.get(DcMotor.class, "back_right");
        susan = hardwareMap.get(DcMotor.class, "carousel");
        thera = hardwareMap.get(DcMotor.class, "binderClip");

//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
        susan.setDirection(DcMotor.Direction.FORWARD);
        thera.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Initialized", "Ready to start");
        waitForStart();

//        while (opModeIsActive()) {
        susan.setPower(0.5);
        thera.setPower(0.5);
        sleep(1000);
        susan.setPower(0);
        thera.setPower(0);
//        }
    }
}
