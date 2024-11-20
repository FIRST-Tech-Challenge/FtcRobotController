package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "testingtalktuah7")
public class testingsigma1 extends LinearOpMode {
    private DcMotor frontLeftMotor = null, backLeftMotor = null;
    private DcMotor frontRightMotor = null, backRightMotor = null;
//    private DcMotor slideExtension = null;

//    private DcMotor slideAbduction = null;

    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private double intakePower = 0;

    // THE SENSOR
    private ColorSensor sensor  = null;


    @Override
    public void runOpMode() {

        //initializing hardware

        //Drive Train motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        //DcMotors for Linear slide
//        slideExtension = hardwareMap.get(DcMotor.class, "slideExtend");
//        slideAbduction = hardwareMap.get(DcMotor.class, "slideAbd");
//
//        slideExtension.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        slideAbduction.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //Takers
        leftIntake = hardwareMap.get(CRServo.class, "l_intake");
        rightIntake = hardwareMap.get(CRServo.class, "r_intake");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

//    MaybeIntake = hardwareMap.get(DcMotor.class, "intake");
        //Setting the direction for the motor on where to rotate

        //Orientation for drivetrain
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //claw
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        //linear slide
//        slideExtension.setDirection(DcMotor.Direction.FORWARD);
//        slideAbduction.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

      /*
        GamePad Map
        GamePad 1 (Driver)
          Left JoyStick = lateral, diagonal, forwards and backwards movements
          Right JoyStick = Rotation of drive train
        GamePad 2 (Operator)
          Button A = toggle position of claw to open or closed (We start closed)
          left stick x = slide extension
          right stick y = slide abduction
       */

            // linear slide controls
            double slideExtendPower = gamepad2.left_stick_y;
            double slideAbdPower = gamepad2.right_stick_y;

            // drive train controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //linear slide power
            double ext = gamepad2.left_stick_x;
            double abt  = gamepad2.right_stick_y;

            //input: theta and power
            //theta is where we want the direction the robot to go
            //power is (-1) to 1 scale where increasing power will cause the engines to go faster
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            //max variable allows to use the motors at its max power with out disabling it
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFront = power * cos / max + turn;
            double rightFront = power * cos / max - turn;
            double leftRear = power * sin / max + turn;
            double rightRear = power * sin / max - turn;

            //Prevents the motors exceeding max power thus motors will not seize and act sporadically
            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + turn;
                rightFront /= power - turn;
                leftRear /= power + turn;
                rightRear /= power - turn;
            }

            //if A on the controller is pressed it will check if the claw is closed
            // HOTFIX: A is open, B is close
            if (gamepad2.a) {
                intakePower = 1;
            } else if(gamepad2.b){
                intakePower = 0;
            }

            slideAbdPower = abt;
            slideExtendPower = ext;

            // Power to the wheels
            frontLeftMotor.setPower(leftFront);
            backLeftMotor.setPower(leftRear);
            frontRightMotor.setPower(rightFront);
            backRightMotor.setPower(rightRear);

            // Power to the arm
//            slideAbduction.setPower(slideAbdPower);
//            slideExtension.setPower(slideExtendPower);

            // Power to the intake
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);

            //Telemetry
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Alpha", sensor.alpha());
            telemetry.addData("Red  ", sensor.red());
            telemetry.addData("Green", sensor.green());
            telemetry.addData("Blue ", sensor.blue());
            telemetry.addData("Intake power: ", intakePower);
            telemetry.update();

        }
    }
}
