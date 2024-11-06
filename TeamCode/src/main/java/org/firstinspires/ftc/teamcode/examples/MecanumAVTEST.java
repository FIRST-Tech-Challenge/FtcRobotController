package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumAVTEST {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RRMotor;
    private DcMotor LRMotor;

    public MecanumAVTEST(HardwareMap hardwareMap) {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");

        // Set motor directions
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RRMotor.setDirection(DcMotor.Direction.REVERSE);
        LRMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double rotate) {
        RFMotor.setPower(forward - strafe - rotate);
        LFMotor.setPower(forward + strafe + rotate);
        RRMotor.setPower(forward + strafe - rotate);
        LRMotor.setPower(forward - strafe + rotate);

    }
}





