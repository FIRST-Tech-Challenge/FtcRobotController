package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; //hello

import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.poleAlignment;
import org.firstinspires.ftc.teamcode.SignalEdgeDetector;

@Config
@TeleOp(name =
        "JacobFumbledTheBag")
public class PP_MecanumTeleOp extends OpMode
{
    //"MC ABHI IS ON THE REPO!!!"
    public final double TURN_PRECESION = 0.65;

    // Declaring drivetrain motors
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    SignalEdgeDetector gamepad2_dpad_up = new SignalEdgeDetector(() -> gamepad2.dpad_up);
    SignalEdgeDetector gamepad2_dpad_down = new SignalEdgeDetector(() -> gamepad2.dpad_down);
    SignalEdgeDetector gamepad2_A = new SignalEdgeDetector(() -> gamepad2.a);
    SignalEdgeDetector gamepad2_B = new SignalEdgeDetector(() -> gamepad2.b);
    SignalEdgeDetector gamepad2_X = new SignalEdgeDetector(() -> gamepad2.x);
    SignalEdgeDetector gamepad2_Y = new SignalEdgeDetector(() -> gamepad2.y);

    SignalEdgeDetector gamepad1_X = new SignalEdgeDetector(() -> gamepad1.x);

    // Declaring mechanism objects
    private Arm armControl;
    private Slide slideControl;
    private Claw clawControl;

    private poleAlignment alignmentControl;

    private GamepadEx driverOp;

    private final double PRECISIONREDUCTION = 0.39;
    /**
     * Get the maximum absolute value from a static array of doubles
     *
     * @param input the input array of double values
     * @return the maximum value from the input array
     */
    private double getMax(double[] input)
    {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    @Override
    public void init()
    {
        driverOp = new GamepadEx(gamepad2);
        driverOp.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER);

        // Expansion Hub Pins
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 2 -> pin 3
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL"); // Pin 1

        // Control Hub Pins
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR"); // Pin 3
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR"); // Pin 2

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Running without an encoder allows us to plug in a raw value rather than one that is proportional
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // to the motors total power. Ex. motor.setPower(0.5); would equal 50% if you run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Running without an encoder does NOT disable encoder counting

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawControl = new Claw(hardwareMap, () -> gamepad2.right_bumper, () -> gamepad2.a);
        OdoPod odoControl = new OdoPod(hardwareMap);
        alignmentControl=new poleAlignment(hardwareMap);
        OdoPod.retract();
    }// INIT()

    @Override
    public void loop()
    {
        boolean precisionToggle = gamepad1.right_trigger > 0.1; // we want to check this every time the loop runs
        //boolean slideUp = gamepad2.right_trigger > 0.1;
        //boolean slideDown = gamepad2.left_trigger > 0.1;
        drive(precisionToggle);
        arm();
        claw();
        slides();
        poleAlignment();


        slideControl.update(telemetry);
        armControl.update(telemetry);
        gamepad2_dpad_up.update();
        gamepad2_A.update();
        gamepad2_B.update();
        gamepad2_X.update();
        gamepad2_Y.update();
        gamepad1_X.update();
        gamepad2_dpad_down.update();
    }// end of loop()

    //        BOT METHODS       \\
    public void drive(boolean precisionToggle)
    {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x*0.75;
        if(precisionToggle)
            rx *= TURN_PRECESION;

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

        if (precisionToggle) {
            motorFrontLeft.setPower(frontLeftPower * PRECISIONREDUCTION);
            motorBackLeft.setPower(backLeftPower * PRECISIONREDUCTION);
            motorFrontRight.setPower(frontRightPower * PRECISIONREDUCTION);
            motorBackRight.setPower(backRightPower * PRECISIONREDUCTION);
        } else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }// end of drive()
    public void poleAlignment()
    {
        if(gamepad1_X.isRisingEdge())
            alignmentControl.toggleAlignmentDevice();
    }

    public void arm(){

        // BUTTONS \\/
        if (gamepad2_Y.isRisingEdge()) {
            armControl.setExtake();
            slideControl.setHighJunction(telemetry);
            clawControl.toggleWristRotate();
        }
        else if (gamepad2_B.isRisingEdge()) {
            armControl.setExtake();
            slideControl.setMidJunction();
            clawControl.toggleWristRotate();

        }
        else if (gamepad2_A.isRisingEdge()) {
            slideControl.setLowJunction();
        }
        else if (gamepad2_X.isRisingEdge()){
            clawControl.wristJoint.setPosition(clawControl.WRIST_INTAKE_POSITION);
            clawControl.wristInExtakePosition = false;
            armControl.setIntake();
            slideControl.setIntakeOrGround();
        }
    }

    public void claw() {
        clawControl.update(); // updates the isOpen and isIntakePosition signal edge detector
        clawControl.toggleOpenClose();
    }

    public void slides(){

        if (gamepad2.right_trigger>0.2){
            slideControl.setManualSlide(120);
        }

        if(gamepad2.left_trigger>0.2){
            slideControl.setManualSlide(-120);
        }

    }

}

