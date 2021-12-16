package org.firstinspires.ftc.teamcode.CompBotW1;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.driveUntilMechStop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CompBotW1Attachments extends CompBotW1 {
    public DcMotor intake = null, lift0 = null, lift1 = null;

    public Servo bucket0, bucket1;
    public CRServo spin0, spin1;

    public final static int liftSafeAdder = 0;
    public int[] liftZero = new int[2];
    public int[] liftSafe = new int[2];
    public DcMotor[] lift;

    private int[] liftHold = new int[2];
    private boolean holding = false;

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
        lift0 = hardwareMap.get(DcMotor.class, "lift0");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        bucket0 = hardwareMap.get(Servo.class, "bucket0");
        bucket1 = hardwareMap.get(Servo.class, "bucket1");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);
        spin0.setPower(0);
        spin1.setPower(0);
        lift0.setPower(0);
        lift1.setPower(0);
        resetLift();

        lift = new DcMotor[]{lift0, lift1};
    }

    public void stop() {
        super.stop();
        intake.setPower(0);
        lift0.setPower(0);
        lift1.setPower(0);
    }
    public void resetLift() {
        lift0.setPower(0);
        lift1.setPower(0);
        liftZero = new int[]{lift0.getCurrentPosition(), lift1.getCurrentPosition()};
        liftSafe = new int[]{liftZero[0]+liftSafeAdder, liftZero[1]+liftSafeAdder};
    }
    public int getLiftPos() {
        return (int)((lift0.getCurrentPosition()-liftZero[0] + lift1.getCurrentPosition()-liftZero[1])/2);
    }
    public void setLiftPower(double p) {
        for(int i = 0; i < 2; i++) {
            if(lift[i].getCurrentPosition() > liftZero[i]) {
                lift[i].setPower(p);
            }
        }
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
    public boolean isLiftSafe() {
        if (lift0.getCurrentPosition() > liftSafe[0] && lift1.getCurrentPosition() > liftSafe[1]) {
            return true;
        }
        return false;
    }
    // Run this in loop
    public void poweredHoldCycle() {
        if(!holding) {
            liftHold = new int[]{lift0.getCurrentPosition(), lift1.getCurrentPosition()};
            holding = true;
        } else {
            for(int i = 0; i < 2; i++) {
                lift[i].setPower(-0.03*(lift[i].getCurrentPosition()-liftHold[i]));
            }
        }
    }
    public void stopPoweredHold() {
        if(holding) {
            for(DcMotor m : lift) {
                m.setPower(0);
            }
        }
        holding = false;
    }
    public void goLiftSafe() {
        goLiftPosition(liftSafeAdder, 1);
    }

    public void setBucket(double position) {
        bucket0.setPosition(position);
        bucket1.setPosition(1-position);
    }

}