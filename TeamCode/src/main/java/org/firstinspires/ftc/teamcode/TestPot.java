package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Autonomous
@Config
public class TestPot extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotorEx slidesJoint;
    DcMotorEx slides; // Clockwise
    Servo claw;

    public static double jointPos = 0;
//MEET NOVEMEBR POTENTIONAL CODE
    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    public static double spinnyWheelsTargetPower = 0;

    public static double speedDivider = 2;

    // Slide hard stops
    public static int maxSlidePos = 4800;
    public static int minSlidePos = 0;

    // PID coefficients
    public static double Kp = 0.019;   // Proportional Gain
    public static double Ki = 0.00015;  // Integral Gain
    public static double Kd = 0.00001;  // Derivative Gain

    private double setpoint = 0;   // PID target position
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime;

    // Predefined positions for the arm
    public static int armInitPos = 0;
    public static int armIntakePos = 735;
    public static int armDeliverPos = 460;
    public static int armDriveAroundPos = 170;

    boolean pidac = false;

    // Predefined positions for the arm
    public static int slidesJInitPos = 0;
    public static int slidesJIntakePos = 850;
    public static int slidesJDeliverPos = 4800;

    // PID variables
    private double targetPosition;
    private double error;
    private double previousError;
    private double integral;
    private double derivative;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        int slidesTargetPos = 0;
        slides.setTargetPosition(slidesTargetPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;
        lastTime = System.currentTimeMillis();

        claw.setPosition(0.3);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        // Setting up PID loop and other actions
        pidac = true;
        // Bring slides up
        setpoint = 1500;

        // Drive forward to the sub
        drive(949.08, 949.08, 949.08, 949.08, 0.4);

        // Put specimen
        claw.setPosition(0.6);  // Opening the claw

        // Extend slides
        slides.setTargetPosition(2000);

        // Bring slides up
        setpoint = 2500;

        // Move backwards
        drive(-200, -200, -200, -200, 0.4);

        // Bring slides down
        setpoint = 1500;
        slides.setTargetPosition(400);

        // Strafe left
        drive(2102.8, -2102.8, 2102.8, -2102.8, 0.4);

        // Pick sample
        executeStatementFor(3000, () -> {
            setpoint = 50;
            claw.setPosition(0.3);  // Close claw
        });

        // Turn left
        setpoint = 900;
        drive(-1321.6, 1321.6, 1321.6, -1321.6, 0.4);

        // Drive forward
        setpoint = 1700;
        drive(700, 700, 700, 700, 0.4);

        // Drop sample
        slides.setTargetPosition(4800);
        executeStatementFor(3000, () -> {
            setpoint = 1700;
            claw.setPosition(0.6);  // Open claw to drop sample
        });

        setpoint = 1800;
        slides.setTargetPosition(1000);

        // Drive back
        drive(-700, -700, -700, -700, 0.4);

        // Turn right
        drive(944, -944, -944, 944, 0.4);

        // Drive forward
        drive(400, 400, 400, 400, 0.4);

        // Pick sample
        executeStatementFor(3000, () -> {
            setpoint = 50;
            claw.setPosition(0.3);  // Close claw
        });

        setpoint = 900;

        // Drive back
        drive(-400, -400, -400, -400, 0.4);

        // Turn left
        drive(-944, 944, 944, -944, 0.4);

        // Drive forward
        setpoint = 1500;
        drive(700, 700, 700, 700, 0.4);

        // Put sample in high basket
        slides.setTargetPosition(4800);
        executeStatementFor(3000, () -> {
            setpoint = 1700;
            claw.setPosition(0.6);  // Open claw
        });

        // Drive back
        drive(-300, -300, -300, -300, 0.4);

        // Finish the operation
        telemetry.addData("Completed", "Operation Finished");
        telemetry.update();
    }

    private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed) {
        frontLeftPos += fLeftTarget;
        frontRightPos += fRightTarget;
        backLeftPos += bLeftTarget;
        backRightPos += bRightTarget;

        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            if (pidac) {
                double currentPosition = slidesJoint.getCurrentPosition();
                double error = setpoint - currentPosition;
                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
                lastTime = currentTime;

                // Proportional term
                double pTerm = Kp * error;

                // Integral term
                integralSum += error * deltaTime;
                double iTerm = Ki * integralSum;

                // Derivative term
                double derivative = (error - lastError) / deltaTime;
                double dTerm = Kd * derivative;

                // Calculate final PID output
                double output = Math.max(-1, Math.min(pTerm + iTerm + dTerm, 1));

                // Set motor power based on PID output
                slidesJoint.setPower(output / speedDivider);

                lastError = error;

                // Send telemetry to the dashboard
                telemetry.addData("Target", slidesJoint.getTargetPosition());
                telemetry.addData("Current Position", currentPosition);
                telemetry.addData("Error", error);
                telemetry.addData("PID Output", output);
                telemetry.update();
            }
        }
    }

    public static void executeStatementFor(long durationInMillis, Runnable actions) {
        long startTime = System.currentTimeMillis();  // Record the start time

        // Loop that continues until the specified duration has passed
        while (System.currentTimeMillis() - startTime < durationInMillis) {
            // Execute the actions passed as a Runnable
            actions.run();
        }
    }
}
