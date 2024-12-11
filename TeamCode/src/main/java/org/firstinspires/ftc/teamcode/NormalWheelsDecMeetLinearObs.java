package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Autonomous
@Config

public class NormalWheelsDecMeetLinearObs extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
   // DcMotorEx slidesJoint;
  //  DcMotorEx slides; //clockwise
   // Servo claw;


  //  public static double jointPos = 0;


    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;


  //  public static double spinnyWheelsTargetPower = 0;

  //  public static double speedDivider = 2;

    //slide hard stops
//    public static int maxSlidePos = 4800;
  //  public static int minSlidePos = 0;

    // PID coefficients
    //public static double Kp = 0.019;   // Proportional Gain
   // public static double Ki = 0.00015;    // Integral Gain
    //public static double Kd = 0.00001;    // Derivative Gain

    // Integral and previous error for PID calculation
   // private double setpoint = 0;   // PID target position
   // private double integralSum = 0;
  //  private double lastError = 0;


    // Time tracking for PID calculation
    private long lastTime;

    // Predefined positions for the arm
    //public static int armInitPos = 0;
 //   public static int armIntakePos = 735;
  //  public static int armDeliverPos = 460;
  //  public static int armDriveAroundPos = 170;

    boolean pidac = false;


    // Predefined positions for the arm
   // public static int slidesJInitPos = 0;
  //  public static int slidesJIntakePos = 850;
    //need to figure out highest pos
   // public static int slidesJDeliverPos = 4800;

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
      //  claw = hardwareMap.get(Servo.class, "claw");
      //  slides = hardwareMap.get(DcMotorEx.class, "slides");
      //  slidesJoint = hardwareMap.get(DcMotorEx.class, "slidesJoint");

        //slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  slidesJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slidesJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesJoint.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        int slidesTargetPos = 0;
        //slides.setTargetPosition(slidesTargetPos);
       // slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;
        lastTime = System.currentTimeMillis();



        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        //850 Ticks for 90 degrees
        //9.44 Tick/Degree
        //14.38 Tick/cm
        long startTime = System.currentTimeMillis();
        long elapsedTime = 0; // Time elapsed in milliseconds
        long timeThreshold = 7000;
        //bring slides up
        //drive forward to sub
        drive(1000, 1000, 1000, 1000, 0.4);
        //put specimen
        sleep(2000);
        //move backwards
        drive(-500,500,-500,-500, 0.4);
        //turn 90 degrees right
        drive(850,-850,-850,850, 0.4);
        //drive forward
        drive(1602.8,1602.8,1602.8, 1602.8, 0.4);
        //turn right
        drive(850,-850,-850,850,0.4);
        //drive forward
        drive(420, 420, 420, 420, 0.4);
        //pick specimen
        sleep(2000);
        //drive backward
        drive(-420, -420, -420, -420, 0.4);
        //turn left
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1902, -1902, -1902, -1902, 0.4);
        //turn 90 degrees left
        drive(-850, 850, 850, -850, 0.4);
        //drive forward
        drive(500, 500, 500, 500, 0.4);
        //put specimen
        sleep(2000);
        //move backwards
        drive(-500,500,-500,-500, 0.4);
        //turn 90 degrees right
        drive(850,-850,-850,850, 0.4);
        //drive forward
        drive(1902.8,1902.8,1902.8, 1902.8, 0.4);
        //turn left 90 degrees
        drive(-850, 850, 850, -850,0.4);
        //drive forward
        drive(640,640,640,640, 0.4);
        //turn right 90
        drive(850, -850, -850, 850, 0.4);
        //drive forward
        drive(250, 250, 250, 250, 0.4);
        //turn left 90
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1120, -1120, -1120, -1120, 0.4);
        sleep(2000);
        //drive forwards
        drive(1120, 1120, 1120, 1120, 0.4);
        //turn right 90
        drive(850, -850, -850, 850, 0.4);
        //drive forward
        drive(180, 180, 180, 180, 0.4);
        //turn left 90
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1120, -1120, -1120, -1120, 0.4);
        //drive backwards
        drive(-100, -100, -100, -100, 0.4);




    }
    private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed) {
        // Update the target positions relative to the current positions
        frontLeftPos += fLeftTarget;
        frontRightPos += fRightTarget;
        backLeftPos += bLeftTarget;
        backRightPos += bRightTarget;

        // Set the updated target positions
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);

        // Set the mode to RUN_TO_POSITION so that the motors run until they reach the target
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the speed for all motors
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Wait until all motors reach their target position
        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            idle();
        }

    }

    /*
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

                //double currentPosition = slidesJoint.getCurrentPosition();
              //  double error = setpoint - currentPosition;
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
               // slidesJoint.setPower(output / speedDivider);

                lastError = error;


                // Send telemetry to the dashboard
              //  telemetry.addData("Target", slidesJoint.getTargetPosition());
              //  telemetry.addData("Current Position", currentPosition);
                telemetry.addData("Error", error);
                telemetry.addData("PID Output", output);
                telemetry.update();

            }


            idle();
        }

     */

    }

    //public static void executeStatementFor(long durationInMillis, Runnable actions) {
      //  long startTime = System.currentTimeMillis();  // Record the start time

        // Loop that continues until the specified duration has passed
      //  while (System.currentTimeMillis() - startTime < durationInMillis) {
      //      // Execute the actions passed as a Runnable
       //     actions.run();
    //    }
   // }


