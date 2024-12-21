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

public class NormalWheelsDecMeetLinearNet extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotorEx armbotJoint;
    DcMotorEx armmidJoint;
    DcMotorEx armtopJoint;
    Servo claw;


    public static double jointbotPos = 0;
    public static double jointmidPos = 0;
    public static double jointtopPos = 0;


    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;
    long lastTime;

    //  public static double spinnyWheelsTargetPower = 0;

    //  public static double speedDivider = 2;

    //hard stop
    public static int maxjbPos = 4800;
    public static int minjbPos = 0;
    public static int maxjmPos = 2000;
    public static int minjmPos = 0;
    public static int maxjtPos = 3000;
    public static int minjtPos = 0;


    // PID coefficients
//    public static double Kp = 0.019;   // Proportional Gain
//    public static double Ki = 0.00015;    // Integral Gain
//    public static double Kd = 0.00001;    // Derivative Gain

    // Integral and previous error for PID calculation
    //   private double setpoint = 0;   // PID target position
//    private double integralSum = 0;
    //   private double lastError = 0;


    // Time tracking for PID calculation
    //  private long lastTime;

    // Predefined positions for the arm
//    public static int armInitPos = 0;
//    public static int armIntakePos = 735;
//    public static int armDeliverPos = 460;
//    public static int armDriveAroundPos = 170;

    boolean pidac = false;


    // Predefined positions for the arm
//    public static int slidesJInitPos = 0;
//    public static int slidesJIntakePos = 850;
    //need to figure out highest pos
//    public static int slidesJDeliverPos = 4800;

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
        armbotJoint = hardwareMap.get(DcMotorEx.class, "armbotJoint");
        armmidJoint = hardwareMap.get(DcMotorEx.class, "armmidJoint");
        armtopJoint = hardwareMap.get(DcMotorEx.class, "armtopJoint");

        armmidJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armbotJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armtopJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armmidJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armbotJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtopJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        int slidesTargetPos = 0;
        //slides.setTargetPosition(slidesTargetPos);
        // slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;
        lastTime = System.currentTimeMillis();

        claw.setPosition(0.2);


        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        //400 Ticks for 90 degrees
        //allign end of bottom left third tooth and bottom first right tooth
        //9.44 Tick/Degree
        //43.38 Tick/cm
        long startTime = System.currentTimeMillis();
        long elapsedTime = 0; // Time elapsed in milliseconds
        long timeThreshold = 7000;
        //bring slides up
        //drive forward to sub
        drive(1300, 1300, 1300, 1300, 0.4);
        //put specimen
       // movearmBot(2000,0.5);
       // movearmMid(1000, 0.5);
        //movearmTop(200,0.5);
      //  claw.setPosition(0.2);
        //move backwards
        drive(-300, -300, -300, -300, 0.4);
        //turn left 90 degrees left
        drive(-390, 390, 390, -390, 0.4);
        //drive forward
        drive(1702.8, 1702.8, 1702.8, 1702.8, 0.4);
        //turn right
        drive(400, -400, -400, 400, 0.4);
        sleep(2000);
        //pick sample
        //turn left
        drive(-601.6, 601.6, 601.6, -501.6, 0.4);
        //drive forward
        drive(400, 400, 400, 400, 0.4);
        //drop sample
        //drive back
        drive(-500, -500, -500, -300, 0.4);
        //turn right
        drive(354, -354, -354, 354, 0.4);
        sleep(1000);
        //drive forward
        drive(350, 350, 350, 350, 0.4);
        sleep(1000);
        //pick sample
        //drive back
        drive(-400, -400, -400, -400, 0.4);
        sleep(1000);
        //turn left
        drive(-454, 454, 454, -454, 0.4);
        sleep(1000);
        //drive forward
        drive(450, 450, 450, 450, 0.4);
        //put sample in high basket
        sleep(1000);
        //drop sample
        drive(-300, -300, -300, -300, 0.4);


        // Check if the elapsed time has reached the threshold


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
            idle();
        }
    }
        private void movearmBot( int position, double power){
            armbotJoint.setTargetPosition(position);
            armbotJoint.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            armmidJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armbotJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armtopJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armbotJoint.setPower(power);

            while (armbotJoint.isBusy() && opModeIsActive()) {
                telemetry.addData("armbotPosition", armbotJoint.getCurrentPosition());
                telemetry.update();
            }

            armbotJoint.setPower(0);
        }
        private void movearmMid ( int position, double power){
            armmidJoint.setTargetPosition(position);
            armmidJoint.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            armmidJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armbotJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armtopJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armmidJoint.setPower(power);

            while (armmidJoint.isBusy() && opModeIsActive()) {
                telemetry.addData("armmidPosition", armmidJoint.getCurrentPosition());
                telemetry.update();
            }

            armbotJoint.setPower(0);
        }
        private void movearmTop ( int position, double power){
            armtopJoint.setTargetPosition(position);
            armtopJoint.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            armmidJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armbotJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armtopJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armtopJoint.setPower(power);

            while (armtopJoint.isBusy() && opModeIsActive()) {
                telemetry.addData("armtopPosition", armtopJoint.getCurrentPosition());
                telemetry.update();
            }

            armbotJoint.setPower(0);
        }
        private void armChamber () {
            movearmBot(2000, 0.5);
            movearmMid(1000, 0.5);
            movearmTop(500, 0.5);
        }
        private void armBasket () {
            movearmBot(3000, 0.5);
            movearmMid(1000, 0.5);
            movearmTop(500, 0.5);
        }
    }




