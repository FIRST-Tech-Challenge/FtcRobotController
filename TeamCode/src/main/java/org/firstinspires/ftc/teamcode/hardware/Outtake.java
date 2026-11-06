package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class Outtake {
    RobotHardwareMap robotHardwareMap;

    private double speed;

    private boolean outtakerunning;
    LinearOpMode opMode;

    private boolean restartouttake;
    private boolean waitForIntakeToStopThenRestartOuttake;

    private long timeofrestartouttake;

    // finish the init method

    public Outtake(RobotHardwareMap robotHardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
    }

    public void initialize() {
        PIDFCoefficients pidfCoeficnets = new PIDFCoefficients();
        //robotHardwareMap.outtakeMotorBack1.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardwareMap.outtakeMotorBack2.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakerunning = false;
        restartouttake = false;
        waitForIntakeToStopThenRestartOuttake = false;
        speed = 960;
        robotHardwareMap.outtakeMotorBack1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardwareMap.outtakeMotorBack2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardwareMap.outtakeMotorBack1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardwareMap.outtakeMotorBack2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pidfCoeficnets.f = 0; //  32767 from first, 1900 max velicty
        pidfCoeficnets.p = 40;  // 0.1 * pidfCoeficnets.f  from first
        pidfCoeficnets.i = 3;  // 0.1 * pidfCoeficnets.p from first
        pidfCoeficnets.d = 0; // 0 from first
        robotHardwareMap.outtakeMotorBack1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeficnets);
        robotHardwareMap.outtakeMotorBack2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeficnets);
    }


    public void setSpeed(double speed_in) {
        speed = speed_in;
        if (speed >= 1900) {
            speed = 1900;
        } else if (speed <= 200) {
            speed = 200;
        }
    }

    public void ToggleOuttakeMotor() {
        if (outtakerunning) {
            outtakerunning = false;
        } else {
            restartouttake = true;
            timeofrestartouttake = System.currentTimeMillis();
        }

    }

    public void outtakeon() {
        if (!outtakerunning && !restartouttake && !waitForIntakeToStopThenRestartOuttake) {// only start outtake if its not running
            restartouttake = true;
            timeofrestartouttake = System.currentTimeMillis();
        }
    }

    public void outtakeonNoReverseTransfer() {
        outtakerunning = true;
    }


    // if the intake was on, we need time for it to stop before we turn on the outtake
    public void outtakeonAfterIntake() {
        if (!outtakerunning && !restartouttake && !waitForIntakeToStopThenRestartOuttake) {// only start outtake if its not running
            waitForIntakeToStopThenRestartOuttake = true;
            timeofrestartouttake = System.currentTimeMillis();
        }
    }

    public void outtakeoff() {
        outtakerunning = false;
    }

    public void ControlMotorSpeed() {
        long currenttime;
        PIDFCoefficients pidfCoeficnets;
        opMode.telemetry.addLine(String.format("outtakespeed %6.3f", speed));
        if (waitForIntakeToStopThenRestartOuttake){
            currenttime = System.currentTimeMillis();
            if ((currenttime - timeofrestartouttake) > 500) {
                waitForIntakeToStopThenRestartOuttake = false;
                restartouttake = true;
                timeofrestartouttake = System.currentTimeMillis();
            }
        }
        if (restartouttake) {
            currenttime = System.currentTimeMillis();
            if ((currenttime - timeofrestartouttake) > 500) {
                this.StopCenterTransferServo();
                outtakerunning = true;
                restartouttake = false;
            } else {
                robotHardwareMap.CenterTransferServo.setPower(-1);
            }
        }
        if (outtakerunning) {
            robotHardwareMap.outtakeMotorBack1.setVelocity(speed);
            robotHardwareMap.outtakeMotorBack2.setVelocity(speed);
        } else {
            robotHardwareMap.outtakeMotorBack1.setPower(0);
            robotHardwareMap.outtakeMotorBack2.setPower(0);

        }

        opMode.telemetry.addLine(String.format("outtakevelocity1 %6.3f", robotHardwareMap.outtakeMotorBack1.getVelocity()));
        opMode.telemetry.addLine(String.format("outtakevelocity2 %6.3f", robotHardwareMap.outtakeMotorBack2.getVelocity()));

        pidfCoeficnets = robotHardwareMap.outtakeMotorBack1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addLine(String.format( pidfCoeficnets.toString()));
    }

    public void increasemotorspeed() {
        speed = speed + 20;
        if (speed >= 1900) {
            speed = 1900;
        }
    }

    public void decreasemotorspeed() {
        speed = speed - 20;
        if (speed < 200) {
            speed = 200;
        }
    }

    public void RunSideTransferServo() {
        robotHardwareMap.LeftTransferServo.setDirection(CRServo.Direction.FORWARD);
        robotHardwareMap.RightTransferServo.setDirection(CRServo.Direction.FORWARD);

        robotHardwareMap.LeftTransferServo.setPower(-1);
        robotHardwareMap.RightTransferServo.setPower(1);
    }

    public void StopSideTransferServo() {
        robotHardwareMap.LeftTransferServo.setPower(0);
        robotHardwareMap.RightTransferServo.setPower(0);
    }

    public void RunCenterTransferServer() {
        robotHardwareMap.CenterTransferServo.setDirection(CRServo.Direction.FORWARD);
        robotHardwareMap.CenterTransferServo.setPower(1);
    }

    public void StopCenterTransferServo() {
        robotHardwareMap.CenterTransferServo.setPower(0);
    }


    public double getspeed(){
        return speed;
    }

    public void blockingShoot(){
        ControlMotorSpeed();
        //can we tell if the outtake is at speed?

        RunSideTransferServo();
        RunCenterTransferServer();
        opMode.sleep(3500);
        StopSideTransferServo();
        StopCenterTransferServo();
    }
}




