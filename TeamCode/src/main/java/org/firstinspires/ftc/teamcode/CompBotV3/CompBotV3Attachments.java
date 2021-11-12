package org.firstinspires.ftc.teamcode.CompBotV3;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CompBotV3Attachments extends CompBotV3 {
    public DcMotor intake = null, spin = null, lift = null;

    public CRServo bucket;

    public CompBotV3Attachments() {
        super();
    }

    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        thisInit(hardwareMap);
    }
    public void init(HardwareMap hardwareMap, boolean cameraInit, Telemetry telemetry) {
        if(cameraInit) {
            super.init(hardwareMap, cameraInit, telemetry);
        }
        thisInit(hardwareMap);
    }
    public void thisInit(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        spin = hardwareMap.get(DcMotor.class, "spin");
        lift = hardwareMap.get(DcMotor.class, "lift");
        bucket = hardwareMap.get(CRServo.class, "bucket");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);
        spin.setPower(0);
        lift.setPower(0);

        bucket.setPower(0);
    }

    public void stop() {
        super.stop();
        intake.setPower(0);
        spin.setPower(0);
        lift.setPower(0);
    }

    public void PhysicsLiftDetection() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}