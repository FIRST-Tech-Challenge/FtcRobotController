package org.firstinspires.ftc.teamcode.CompBotW1;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.driveUntilMechStop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CompBotW1Attachments extends CompBotW1 {
    public DcMotor intake = null, lift0 = null, lift1 = null;

    public CRServo bucket, spin0, spin1;

    public int[] liftZero = new int[2];

    public CompBotW1Attachments() {
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
    public void init(HardwareMap hardwareMap, boolean cameraInit, Telemetry telemetry, String color) {
        if(cameraInit) {
            super.init(hardwareMap, cameraInit, telemetry, color);
        }
        thisInit(hardwareMap);
    }
    public void thisInit(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        spin0 = hardwareMap.get(CRServo.class, "spin0");
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        // lift0 = hardwareMap.get(DcMotor.class, "lift1");
        // lift1 = hardwareMap.get(DcMotor.class, "lift2");
        // bucket = hardwareMap.get(CRServo.class, "bucket");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lift0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // lift0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);
        spin0.setPower(0);
        spin1.setPower(0);
        // lift0.setPower(0);
        // lift1.setPower(0);
        // bucket.setPower(0);

        // resetLift();
    }

    public void stop() {
        super.stop();
        intake.setPower(0);
        spin0.setPower(0);
        spin1.setPower(0);
        lift0.setPower(0);
        lift1.setPower(0);
    }
    /*
    public void resetLift() {
        int[] initPos = {lift0.getCurrentPosition(), lift1.getCurrentPosition()};
        lift0.setPower(-1);
        lift1.setPower(-1);
        ElapsedTime dt = new ElapsedTime();
        ElapsedTime minTime = new ElapsedTime();
        double[] v;
        boolean finished = false;
        while(true) {
            int[] cPos = {lift0.getCurrentPosition(), lift1.getCurrentPosition()};
            v = new double[]{(double) (cPos[0] - initPos[0]) / dt.milliseconds(), (double) (cPos[1] - initPos[1]) / dt.milliseconds()};
            for(double vel : v) {
                if (Math.abs(vel) < 0.2 && minTime.milliseconds() > 2000) {
                    finished = true;
                }
            }
            if(finished){
                break;
            }
            dt.reset();
            initPos = cPos;
        }
        lift0.setPower(0);
        lift1.setPower(0);
        liftZero = new int[]{lift0.getCurrentPosition(), lift1.getCurrentPosition()};
    }
    public int getLiftPos() {
        return (int)((lift0.getCurrentPosition()-liftZero[0] + lift1.getCurrentPosition()-liftZero[1])/2);
    }
    public void setLiftPower(double p) {
        lift0.setPower(p);
        lift1.setPower(p);
    }
    public void goLiftPosition(int ticksToDrive, double speed) {
        lift0.setTargetPosition(lift1.getCurrentPosition() + ticksToDrive);
        lift0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setTargetPosition(lift1.getCurrentPosition() + ticksToDrive);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(lift0.isBusy() || lift1.isBusy()) {
            lift0.setPower(speed);
            lift1.setPower(speed);
        }
        lift0.setPower(0);
        lift0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setPower(0);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
     */

}