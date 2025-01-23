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
    private static final double ticksPerInch = (double) 1425/(Math.PI * 3.75); // TODO: wtf???
    private static final double Max_Extension = 22 * ticksPerInch; // Horizontal Max Ticks
    private static final double degPerTick = (double) 360/1425;
    // PID constants
    public double sP = 0.0009;
    public double sI = 0.000;
    public double sD = 0.00;
    public double pP = 0.004;
    public double pI = 0;
    public double pD = 0.00002;
    public double pCos = 0.325;
    public double pExt = 0.000075;
    boolean isTargAngDown = false;
    PIDFController slidePID;
//    ElapsedTime pidTimer;
    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private double targSlideHeight = 0;
    private int targetSlidePosition = (int) (targSlideHeight * ticksPerInch);  // Starting at initial position (adjust)
    private int targPivotPos;
    private int maxSlidePos;
    private int maxPivotPos = -350;
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

    public void loop(double P, double I, double D) {
        targPivotPos = Math.min(0, Math.max(targPivotPos, maxPivotPos));
        // Ensure the motor does not exceed max forward extension
        double out = slidePID.calculate(slideMotor.getCurrentPosition(), targetSlidePosition);
//        SlewRateLimiter slideLimiter = new SlewRateLimiter(0.5); //TODO: Tune this
        slidePID.setPIDF(P, I, D,0);
        slideMotor.setPower(out);
        pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
        pivot.setPower(pivotPower); // use updatdePID to fix this
    }
    public double getHypotenuseMax(double height) {
        double in_inch;
        in_inch=Math.sqrt(height*height+22*22);
        return in_inch*ticksPerInch;
    }
    public double getHypotenuseSpecimen(double height) {
        double in_inch;
        in_inch=Math.sqrt(height*height+10*10);
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
        isTargAngDown = false;
        targPivotPos = -15;
        targSlideHeight = inches;
        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
    }
    public void goOut(double inches) {
//        isTargAngDown = false;
//        targSlideHeight = (1/ticksPerInch) * getHypotenuseMax(inches);
//        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
//        targPivotPos = (int) (-5 - (pivAng * 105/190));
//        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
//
        targetSlidePosition = 3600;
        targPivotPos = -150;
    }
    public void goSpecimen(double inches) {
//        isTargAngDown = false;
//        targSlideHeight = (1/ticksPerInch) * getHypotenuseSpecimen(inches);
//        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
//        targPivotPos = (int) (-5 - (pivAng * 105/190));
//        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
        targPivotPos = -80;
        targetSlidePosition = 2000;
    }
    public void goSpecimenDown(double inches) {
//        isTargAngDown = false;
//        targSlideHeight = (1/ticksPerInch) * getHypotenuseSpecimen(inches);
//        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
//        targPivotPos = (int) (-5 - (pivAng * 105/190));
//        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
        targPivotPos = -80;
        targetSlidePosition = 1800;
    }
    public void goGround(double inches) {

        targPivotPos = -345;
        targetSlidePosition = 1500;
        isTargAngDown = true;
        wristOut();
    }
    public boolean isAtTargetHeight() {
        if (Math.abs(slideMotor.getCurrentPosition()/ticksPerInch - targSlideHeight) < 2) {
            return true;
        } else {
            return false;
        }
    }
    public boolean isAtTargAng() {
        if (Math.abs(pivot.getCurrentPosition() - targPivotPos) < 10) {
            return true;
        } else {
            return false;
        }
    }
    // Initialize motor
    public void manualOut() {
        targetSlidePosition += 10;
    }
    public void manualIn() {
        targetSlidePosition -= 10;
    }
    public void manualUp() { targPivotPos++;}
    public void manualDown() { targPivotPos--;}
    public void grab() { claw.setPosition(0); }
    public void release() {claw.setPosition(0.12);}
    public void wristOut() {wrist.setPosition(.25);}
    public void wristIn() {wrist.setPosition(.9);}
    public void wristDown() {wrist.setPosition(.5);}



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
        double ff;
        double output;
        if (isTargAngDown) {
            if (pivot.getCurrentPosition() >= -335) {
                ff = ((pExt * slideMotor.getCurrentPosition()) - pCos * Math.cos(Math.toRadians(85 - (current * degPerTick))));
                output = (error * 0.0015) + ff;
            } else {
             output = 0;
            }
        } else {
            ff = ((pExt * slideMotor.getCurrentPosition()) - pCos * Math.cos(Math.toRadians(85 - (current * degPerTick))));
            output = (error * pP) + (integral * pI) + (derivative * pD) + (ff);
        }

        pivotTimer.reset();
        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
