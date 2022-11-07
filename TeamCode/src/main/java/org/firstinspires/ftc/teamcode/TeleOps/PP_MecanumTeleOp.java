package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmClass;

@Config
@TeleOp(name = "PP_MecanumTeleOp") // This is redundant, the name will automatically be set to the class name if you don't set it - Tiernan
public class PP_MecanumTeleOp extends OpMode {
    //"MC ABHI IS ON THE REPO!!!"

    public static double armKp = 0.0;
    public static double armKd = 0.0;
    public static double armKi = 0.0;
    public static double armKf = 0.0;

    // Declaring class members to be used in other methods
    private ElapsedTime runtime = new ElapsedTime();
    PIDController motorPID = new PIDController(armKp, armKd, armKi, armKf, runtime);
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, slideMotorLeft, slideMotorRight, armMotor;
    private Servo clawJoint, wristJoint;


    // TODO: Why are these not private, this makes me sad. Also you could make them static and stick in their own class... - Tiernan
    // Claw Servo open and close constants

    final double[] servo_MinMax = {0, 0.5};
    final double OPEN = 0.45; // claw open
    final double CLOSE = 0.35; // claw close
    ArmClass armControl;

    /**
     * Get the maximum absolute value from a static array of doubles
     *
     * @param input the input array of double values
     * @return the maximum value from the input array
     */
    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    @Override
    public void init() {
        // Expansion Hub Pins
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 2
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL"); // Pin 1
        slideMotorLeft = (DcMotorEx) hardwareMap.dcMotor.get("SL"); // Pin 0

        // Control Hub Pins
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR"); // Pin 3
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR"); // Pin 2
        slideMotorRight = (DcMotorEx) hardwareMap.dcMotor.get("SR"); // Pin 1
        armMotor = (DcMotorEx) hardwareMap.dcMotor.get("ARM"); // Pin 0

        clawJoint = hardwareMap.get(Servo.class, "CLAW"); // Pin 1
        wristJoint = hardwareMap.get(Servo.class, "WRIST"); // Pin 0


        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Running without an encoder allows us to plug in a raw value rather than one that is proportional
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // to the motors total power. Ex. motor.setPower(0.5); would equal 50% if you run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Running without an encoder does NOT disable counting

        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // You don't want to run using encoders for PID
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        clawJoint.setPosition(OPEN);

        armControl = new ArmClass(motorPID, slideMotorLeft, slideMotorRight, armMotor, wristJoint, clawJoint, OPEN, CLOSE);
    }// end of init

    @Override
    public void loop() {
        motorPID.setCoefficients(armKp, armKi, armKd, armKf);
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        drive(precisionToggle);
        arm(); // includes claw mechanism
    }


    //        BOT METHODS       \\
    public void drive(boolean precisionToggle) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the mecanum motor powers
        double frontLeftPower = (y + x + 2 * rx) / denominator;
        double backLeftPower = (y - x + 2 * rx) / denominator;
        double frontRightPower = (y - x - 2 * rx) / denominator;
        double backRightPower = (y + x - 2 * rx) / denominator;


        // Cube the motor powers
        frontLeftPower = Math.pow(frontLeftPower, 3);
        frontRightPower = Math.pow(frontRightPower, 3);
        backLeftPower = Math.pow(backLeftPower, 3);
        backRightPower = Math.pow(backRightPower, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        });

        // Resize the motor power values
        if (maxValue > 1) {
            frontLeftPower /= maxValue;
            frontRightPower /= maxValue;
            backLeftPower /= maxValue;
            backRightPower /= maxValue;
        }

        // Hi!
        if (precisionToggle) {
            motorFrontLeft.setPower(frontLeftPower * 0.3);
            telemetry.addData("Power front left",frontLeftPower*0.3);
            motorBackLeft.setPower(backLeftPower * 0.3);
            telemetry.addData("Power back left",backLeftPower*0.3);
            motorFrontRight.setPower(frontRightPower * 0.3);
            telemetry.addData("Power front right",frontRightPower*0.3);
            motorBackRight.setPower(backRightPower * 0.3);
            telemetry.addData("Power back right:",backRightPower*0.3);
        } else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }

    public void arm() {
        // BUTTONS \\
        if (gamepad2.a) {
            // 1000 represents an arbitrary value for the max -> 700 mid, 400 low, 0 ground
            armControl.goToJunction(1000);
        } else if (gamepad2.b) {
            armControl.goToJunction(700);
        } else if (gamepad2.y) {
            armControl.goToJunction(400);
        } else if (gamepad2.x) {
          armControl.goToJunction(0);
        }

        // DPAD \\
        if (gamepad2.dpad_left) {
            armControl.manualClawRotate(-0.01);
        } else if (gamepad2.dpad_right) {
            armControl.manualClawRotate(0.01);
        } else if (gamepad2.dpad_up) {
            telemetry.addLine("dpadUp");
            armControl.armPivot(100); // pivots up
        } else if (gamepad2.dpad_down) {
            telemetry.addLine("dpadDown");
            armControl.armPivot(-100); // pivots down
        }

        // BUMPER \\
        if (gamepad2.right_bumper) {
            armControl.claw();
            //clawJoint.setPosition(0.4); <-- still does not work
        }

        // TRIGGERS \\
        if (gamepad2.right_trigger > 0.2) {
           armControl.manualSlides(5);
        } else if (gamepad2.left_trigger > 0.2) {
           armControl.manualSlides(-5);
        }
    }
}

