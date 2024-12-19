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
    private static final double ticksPerInch = (double) 1024 /24; // TODO: wtf???
    private static final double Max_Extension = 1024; // Max safe extension distance from slide base
    private static final double degPerTick = (double) 360 /752;
    // PID constants
//    public static double sP = 0.01;
//    public static double sI = 0.0001;
//    public static double sD = 0.001;
    public double pP = 0;
    public double pI = 0;
    public double pD = 0;
    public double pCos = 0.4;
    public double pExt = 3e-4;
//    PIDFController pivotPID;
//    ElapsedTime pidTimer;
    public static double maxSlidePower = 0;
    private boolean manualMode = true;
    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private double targSlideHeight = 0;
    private int targetSlidePosition = (int) (targSlideHeight * ticksPerInch);  // Starting at initial position (adjust)
    private int targPivotPos;
    private int maxSlidePos;
    private int maxPivotPos = -210;
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
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets the STARTING pos to 0
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // use inbuilt pid
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // don't "try its best to run at a velocity" do it ourselves
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxSlidePos = (int) Math.round(Max_Extension);
        targSlideHeight = 0;
        targPivotPos = 0;
        pivotTimer = new ElapsedTime();
//        pivotPID = new PIDController(pP, pI, pD);
        grab();
        wristOut();
    }

    public void loop() {
//
        if (opMode.gamepad1.start && opMode.gamepad1.back) {
            manualMode = false;
        } else if (opMode.gamepad1.start && opMode.gamepad1.x) {
            manualMode = true;
        }
        if (manualMode) {
            if (opMode.gamepad1.dpad_up) {
                claw.setPosition(claw.getPosition() + 0.0025);
            } else if (opMode.gamepad1.dpad_down) {
                claw.setPosition(claw.getPosition() - 0.0025);
            } else if (opMode.gamepad1.dpad_left) {
                wrist.setPosition(wrist.getPosition() + 0.0025);
            } else if (opMode.gamepad1.dpad_right) {
                wrist.setPosition(wrist.getPosition() - 0.0025);
            }
            if (opMode.gamepad1.x) {
                manualIn();
            } else if (opMode.gamepad1.b) {
                manualOut();
            } else if (opMode.gamepad1.y) {
                manualUp();
            } else if (opMode.gamepad1.a) {
                manualDown();
            }
        } else {
            if (opMode.gamepad1.a) {
                setSlideHeight(0);
            } else if (opMode.gamepad1.x) {
                setSlideHeight(13.0);
            } else if (opMode.gamepad1.b) {
                setSlideHeight(26.0 - 5);
            } else if (opMode.gamepad1.y) {
                setSlideHeight(43.0 - 5);
            }
            if (opMode.gamepad1.left_bumper) {
                targPivotPos = -75;
            } else if (opMode.gamepad1.right_bumper) {
                targPivotPos = -195;
            } else if (opMode.gamepad1.right_trigger > 0.5) {
                targPivotPos = -5;
            }
            if (opMode.gamepad1.dpad_up) {
                wristOut();
            } else if (opMode.gamepad1.dpad_down) {
                wristIn();
            } else if (opMode.gamepad1.dpad_right) {
                grab();
            } else if (opMode.gamepad1.dpad_left) {
                release();
            }
            targetSlidePosition = (int) (targSlideHeight*ticksPerInch);
        }
        targetSlidePosition = Math.max(0, Math.min(targetSlidePosition, maxSlidePos));
        targPivotPos = Math.min(0, Math.max(targPivotPos, maxPivotPos));
        // Ensure the motor does not exceed max forward extension
        slideMotor.setTargetPosition(targetSlidePosition);
        double direction = maxSlidePower * Math.signum((double) targetSlidePosition- slideMotor.getCurrentPosition());
        slideMotor.setPower(direction);
//        pivotPID.setPIDF(aP, aI, aD, aF);
        pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
        pivot.setPower(pivotPower); // use updatdePID to fix this
    }
    public void updatePID(double aP, double aI, double aD, double aCos, double aExt) {
        pP = aP;
        pI = aI;
        pD = aD;
        pCos = aCos;
        pExt = aExt;
    }
    public void setSlideHeight(double inches) {
        targSlideHeight = inches;
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
    public void wristIn() {wrist.setPosition(.79);}



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
        double ff = ((pExt * slideMotor.getCurrentPosition())-pCos*Math.cos(Math.toRadians(90-(current * degPerTick))));
        double output = (error * pP) + (integral * pI) + (derivative * pD) + (ff);
        pivotTimer.reset();
        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
