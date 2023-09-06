package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends Mechanism {
    public DcMotorEx turret;
    private Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();
    //CONSTANTS
    public static double kP = -0.0005;
    public static double kP_sideways = -0.001;
    public static double kD = 0;
    public static double kS = 0.05;
    public static double vMax = 0.6;
    public static double tpd = 1.0; //encoder res * gear ratio / 360 degrees = ticks per degree
    public static double target = 0;
    public double lastError = 0; //separate error for each motor
    public double powers = 0;
    public static double bound = 2; //error bound for turret
    public static double distance = 0;
    public static double incremenet = 4;
    public static double side = 1;
    public static double pick = 413;
    public static double score = 207;
    public static boolean isPicking = false;
    public static boolean cycleMode = false;
    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        turret = hwMap.get(DcMotorEx.class, "hdHexMotor");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition(0);
        timer.reset();
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = turret.getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError) / time - kS * Math.signum(error);
        if(Math.abs(distance) <= 300 && Math.abs(target) == 207) {
            pd = kP_sideways * error + kD * (error-lastError) / time - kS * Math.signum(error);
        }
        lastError = error;
        timer.reset();
        if(Math.abs(error) < bound) {
            pd = 0;
        }
        turret.setPower(Range.clip(pd, -vMax, vMax));
        powers = Range.clip(pd, -vMax, vMax);
    }

    public void loop() {
        update(0);
        telemetry.addData("Turret Posi: ", turret.getCurrentPosition());
    }

    public void setTargetPosition(double pos) {
        target = pos;
    }

    public void setTargetAngle(double angle){ //ANGLE MUST BE IN DEGREES
        target = angle * tpd;
    }

    public double getPos(int motor) {
        return turret.getCurrentPosition();
    }

    public double getAngle() {
        return (getPos(0) + getPos(1))/(2*tpd); //avg of motor pos / ticks per degree
    }

    public void setPower(double power, int motor) {
        turret.setPower(power);
    }

    public void setPower(double power) {
        setPower(power, 0);
        setPower(power, 1);
    }

    public void recalibrate() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
        turret.setPower(0);
    }
    public double getAvgError() {
        return ((target-getPos(0)) + (target-getPos(1))) / 2;
    }

    public double getError(int motor) {
        return target - getPos(motor);
    }

    public void inc(double sign) {
        if(Math.abs(target + Math.signum(sign)*incremenet) <= 600) {
            target += Math.signum(sign) * incremenet;
        }
    }

    public void pick() {
        setTargetPosition(side*pick);
        isPicking = true;
    }

    public void score() {
        if(cycleMode) {
            sideways();
        }else {
            distance = target;
            setTargetPosition(side * score);
            distance -= target;
        }
        isPicking = false;
    }

    public void toggleTurret() {
        if(isPicking) {
            score();
        }else {
            pick();
        }
    }

    public void center() {
        setTargetPosition(0);
        isPicking = false;
    }

    public void down() {
        setTargetPosition(side*413);
        isPicking = false;
    }

    public void sideways() {
        distance = target;
        setTargetPosition(side*207);
        distance -= target;
        isPicking = false;
    }
}
