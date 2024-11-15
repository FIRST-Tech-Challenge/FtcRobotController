package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class nematocyst {
    private DcMotorEx pivot;
    private DcMotorEx slideMotor;

    private static final double RobotBase_to_LS = 8.5;  // Distance from the front of the robot to the slide base
    private static final double Max_Extension = 42; // Max safe extension distance from slide base

    // PID constants
    private double sP = 0.01;
    private double sI = 0.0001;
    private double sD = 0.001;
    private double pP = 0;
    private double pI = 0;
    private double pD = 0;

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private int targetSlidePosition = 0;  // Starting at initial position (adjust)
    private int targAng;
    private int targPivotPos = 0;
    double slidePower;
    private int maxSlidePos;
    private int maxPivotPos;
    double pivotPower;
    OpMode opMode;
    public nematocyst(OpMode opMode) {
        opMode = opMode;
    }
    public void init(String pivotName, String slideName) {
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class, slideName);
        pivot = opMode.hardwareMap.get(DcMotorEx.class, pivotName);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxPivotPos = Max_Extension * ;
    }

    public void loop() {
        if (opMode.gamepad1.dpad_up) {
            manualIn();
        } else if (opMode.gamepad1.dpad_down) {
            manualOut();
        } else if (opMode.gamepad1.y) {
            manualUp();
        } else if (opMode.gamepad1.a) {
            manualDown();
        }
        targetSlidePosition = Math.max(0, Math.min(targetSlidePosition, maxSlidePos));
        targPivotPos = Math.max(0, Math.min(targPivotPos, maxPivotPos));

        // Calculate current forward reach
        double targetExtension = targetSlidePosition + RobotBase_to_LS;

        // Ensure the slide does not exceed max forward extension
        if ((targetExtension < Max_Extension) && targetExtension > RobotBase_to_LS) {
            slideMotor.setTargetPosition(targetSlidePosition);
            double power = calculatePID(targetSlidePosition, slideMotor.getCurrentPosition());
            slideMotor.setPower(power);
        } else {
            slideMotor.setPower(0);  // Stop motor if exceeding extension limits (failsafe)
        }

        if ((targPivotPos > ))
        pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
        pivot.setPower(pivotPower);
    }
    public void updatePID(double slideP, double slideI, double slideD, double aP, double aI, double aD) {
        sP = slideP;
        sI = slideI;
        sD = slideD;
        pP = aP;
        pI = aI;
        pD = aD;
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



            // Clamp target position to a safe range (adjust limits as needed)
    public void getTelemetry() {
        // Telemetry for debugging
        opMode.telemetry.addData("Target Position", targetSlidePosition);
        opMode.telemetry.addData("Current Position", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Slide Power", slidePower);
        opMode.telemetry.update();
    }
    public void getTelemetry(Telemetry t) {
        // Telemetry for debugging
        t.addData("Target Position", targetSlidePosition);
        t.addData("Current Position", slideMotor.getCurrentPosition());
        t.addData("Slide Power", slidePower);
        t.update();
    }

    private double calculatePID(int target, int current) {
        // Calculate error
        double error = target - current;

        // Proportional term
        double pTerm = sP * error;

        // Integral term
        integral += error;
        double iTerm = sI * integral;

        // Derivative term
        double dTerm = sD * (error - lastError);
        lastError = error;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
