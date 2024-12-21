package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Autonomous
@Config

public class BasketSampleAuto extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotorEx slidesJoint;
    DcMotorEx slides; //clockwise
    Servo joint;
    CRServo spinnyWheels;



    public static double jointPos = 0;


    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    public static double  spinnyWheelsTargetPower = 0;

    public static double speedDivider = 2;

    //slide hard stops
    public static int maxSlidePos = 4800;
    public static int minSlidePos = 0;

    // PID coefficients
    public static double Kp = 0.019;   // Proportional Gain
    public static double Ki = 0.00015;    // Integral Gain
    public static double Kd = 0.00001;    // Derivative Gain

    // Integral and previous error for PID calculation
    private double setpoint = 0;   // PID target position
    private double integralSum = 0;
    private double lastError = 0;



    // Time tracking for PID calculation
    private long lastTime;

    // Predefined positions for the arm
    public static int armInitPos = 0;
    public static int armIntakePos = 735;
    public static int armDeliverPos = 460;
    public static int armDriveAroundPos = 170;


    // Predefined positions for the arm
    public static int slidesJInitPos = 0;
    public static  int slidesJIntakePos = 850;
    //need to figure out highest pos
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
        joint = hardwareMap.get(Servo.class, "joint");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");
        spinnyWheels = hardwareMap.get(CRServo.class, "spinnyWheels");

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

        joint.setPosition(0.15);


        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        //850 Ticks for 90 degrees
        //9.44 Tick/Degree
        //14.38 Tick/cm
        long startTime = System.currentTimeMillis();
        long elapsedTime = 0; // Time elapsed in milliseconds
        long timeThreshold = 7000;

        //Strafing Left 10 cm
        drive(143.8, -143.8, 143.8, -143.8, 0.4);
        //Backwards 90 cm
        drive(-1490, -1490, -1490, -1490, 0.4);
        sleep(500);
        //Forward 32 cm
        drive(355.16, 355.16, 355.16, 355.16, 0.7);
        //Strafe Left 119 cm
        drive(900, -900, 900, -900, 0.7);
        //Backwards
        drive(-400, -400, -400, -400, 0.4);
        //Leften
        drive(-605.6, 605.6, 605.6, -605.6, 0.4);
        //Backwards
        drive(200, 200, 200, 200, 0.4);
        //Strafe Left 50 cm
        drive(400.8, -400.8, 400.8, -400.8, 0.4);
        elapsedTime = System.currentTimeMillis() - startTime;

        // Check if the elapsed time has reached the threshold
        if (elapsedTime >= timeThreshold) {
            setpoint = 800;
            joint.setPosition(0.75);
            spinnyWheels.setPower(-0.5);
            setpoint = 735;
            setpoint = 2210;

            drive(-300, -300, -300, -300, 0.4);
            drive(-1700.4, 1700.4, 1700.4, -1700.4, 0.4);
            drive(450, 450, 450, 450, 0.4);
            slides.setTargetPosition(4800);
            spinnyWheels.setPower(0.5);
        }


        while(opModeIsActive() && !isStopRequested()) {
            double currentPosition = slidesJoint.getCurrentPosition();
            double error = setpoint - currentPosition;
            // Calculate time step (delta time)
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

            spinnyWheels.setPower(spinnyWheelsTargetPower);
        }
    }



        private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed ) {
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
            while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {

            }
        }

    }