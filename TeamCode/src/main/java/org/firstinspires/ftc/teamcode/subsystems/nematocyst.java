package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class nematocyst {
    private DcMotor pivot;
    private DcMotor slideMotor;
    private Servo claw;
    private Servo wrist;
    double pivotPower;
//    private static final int degreesToTicks = 1425 /360;
//    private static final double inchesToDegrees =  1/2.067;
    private static final double ticksPerInch = (double) 8192/(Math.PI * 2 * 2.4 / 2.54 ); // TODO: wtf???
    private static final double Max_Extension = 22 * ticksPerInch; // Horizontal Max Ticks
    private static final double degPerTick = (double) 360 /752;
    // PID constants
    public static double sP = 1e-4;
    public static double sI = 0.000;
    public static double sD = 0.00;
    public double pP = 0.00425;
    public double pI = 0;
    public double pD = 8e-6;
    public double pCos = 0.3375;
    public double pExt = 1e-7;
    PIDFController slidePID;
//    ElapsedTime pidTimer;
    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private double targSlideHeight = 0;
    private int targetSlidePosition = (int) (targSlideHeight * ticksPerInch);  // Starting at initial position (adjust)
    private int targPivotPos;
    private int maxSlidePos;
    private int maxPivotPos = -195;
    ElapsedTime pivotTimer;
    OpMode opMode;
    public nematocyst(OpMode OM) {
        opMode = OM;
    }
    public void init(String pivotName, String slideName, String wristName, String clawName) {
        slideMotor = opMode.hardwareMap.get(DcMotor.class, slideName);
        pivot = opMode.hardwareMap.get(DcMotor.class, pivotName);
        claw = opMode.hardwareMap.get(Servo.class, clawName);
        wrist = opMode.hardwareMap.get(Servo.class, wristName);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets the STARTING pos to 0
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // don't "try its best to run at a velocity" do it ourselves
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxSlidePos = (int) Math.round(Max_Extension);
        targSlideHeight = 0;
        targPivotPos = 0;
        pivotTimer = new ElapsedTime();
        slidePID = new PIDController(sP, sI, sD);
        grab();
        wristIn();
    }

    public void loop() {
        if (opMode.gamepad1.a) {
            goUp(0);
        } else if (opMode.gamepad1.x) {
            goGround(13.0);
        } else if (opMode.gamepad1.b) {
            goOut(28);
        } else if (opMode.gamepad1.y) {
            goOut(45.0);
        }
        if (opMode.gamepad1.dpad_up) {
            wristOut();
        } else if (opMode.gamepad1.dpad_down) {
            wristIn();
        } else if (opMode.gamepad1.dpad_right) {
            grab();
        } else if (opMode.gamepad1.dpad_left) {
            release();
        } else if (opMode.gamepad1.left_bumper) {
            wristDown();
        }
        targPivotPos = Math.min(0, Math.max(targPivotPos, maxPivotPos));
        // Ensure the motor does not exceed max forward extension
        double out = slidePID.calculate(slideMotor.getCurrentPosition(), targetSlidePosition);
        slideMotor.setPower(out);
        pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
        pivot.setPower(pivotPower); // use updatdePID to fix this
    }
    public double getHypotenuseMax(double height) {
        double in_inch;
        in_inch=Math.sqrt(height*height+22*22);
        return in_inch*ticksPerInch;
    }
    public void updatePID(double aP, double aI, double aD, double aCos, double aExt) {
        pP = aP;
        pI = aI;
        pD = aD;
        pCos = aCos;
        pExt = aExt;
    }
    public void updateSlidePID(double P, double I, double D) {
        slidePID.setPIDF(P, I, D, 0);
    }
    public void goUp(double inches) {
        targPivotPos = 0;
        targSlideHeight = inches;
        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
    }
    public void goOut(double inches) {
        targSlideHeight = (1/ticksPerInch) * getHypotenuseMax(inches);
        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
        targPivotPos = (int) (-5 - (pivAng * 105/190));
        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
    }
    public void goGround(double inches) {
        targSlideHeight = inches;
        targPivotPos = -190;
        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
        targetSlidePosition = Math.max(0, Math.min(targetSlidePosition, maxSlidePos));
    }
        // Initialize motor
    public void manualOut() {
        targetSlidePosition += 1;
    }
    public void manualIn() {
        targetSlidePosition -= 1;
    }
    public void manualUp() { targPivotPos++;}
    public void manualDown() { targPivotPos--;}
    public void grab() { claw.setPosition(0); }
    public void release() {claw.setPosition(0.12);}
    public void wristOut() {wrist.setPosition(0);}
    public void wristIn() {wrist.setPosition(.75);}
    public void wristDown() {wrist.setPosition(.25);}



            // Clamp target position to a safe range (adjust limits as needed)
    public void getTelemetry() {
        // Telemetry for debugging
        opMode.telemetry.addData("Target Slide Position", targetSlidePosition);
        opMode.telemetry.addData("Current Slide Position", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Targ Slide Height", targSlideHeight);
        opMode.telemetry.addData("CUrrent SLIde Height", slideMotor.getCurrentPosition()/(ticksPerInch));
        opMode.telemetry.addData("Slide Power", slideMotor.getPower());
        opMode.telemetry.addData("Wrist pos", wrist.getPosition());
        opMode.telemetry.addData("Claw pos", claw.getPosition());
        opMode.telemetry.addData("Pivot Targ", targPivotPos);
        opMode.telemetry.addData("Pivot Pos", pivot.getCurrentPosition());
        opMode.telemetry.addData("Pivot Power", pivotPower);
        opMode.telemetry.update();
    }
    public void getTelemetry(Telemetry t) {
        // Telemetry for debugging
        t.addData("Target Slide Position", targetSlidePosition);
        t.addData("Current Slide Position", slideMotor.getCurrentPosition());
        t.addData("Targ Slide Height", targSlideHeight);
        t.addData("CUrrent SLIde Height", slideMotor.getCurrentPosition()/(ticksPerInch));
        t.addData("Slide Power", slideMotor.getPower());
        t.addData("Wrist pos", wrist.getPosition());
        t.addData("Claw pos", claw.getPosition());
        t.addData("Pivot Targ", targPivotPos);
        t.addData("Pivot Pos", pivot.getCurrentPosition());
        t.addData("Pivot Power", pivotPower);
        t.update();
    }
    private double calculatePID(int target, int current) {
        // Calculate error

        double error = target - current;
        integral += (error * pivotTimer.seconds());
//        double iTerm = i * integral;
        // Derivative term
        double derivative = ((error - lastError)/pivotTimer.seconds());
        lastError = error;
        // PID output
        double ff = ((pExt * slideMotor.getCurrentPosition())-pCos*Math.cos(Math.toRadians(85-(current * degPerTick))));
        double output = (error * pP) + (integral * pI) + (derivative * pD) + (ff);
        pivotTimer.reset();
        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
