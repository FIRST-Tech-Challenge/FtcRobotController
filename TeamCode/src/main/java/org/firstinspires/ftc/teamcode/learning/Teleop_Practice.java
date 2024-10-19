package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class Teleop_Practice extends LinearOpMode {

    private DcMotor BackLeftWheel;
    private DcMotor FrontLeftWheel;
    private DcMotor BackRightWheel;
    private DcMotor FrontRightWheel;
    private Servo Extension;
    private Servo DeliveryBox;
    private CRServo IntakeBox;
    public Servo SpecimenGripper;
    private double extensionServoPosition;
    private double deliveryServoPosition;
    private double specimenServoPosition;
    private float gamepad1_RightStickYValue;
    private float gamepad1_RightStickXValue;
    private float gamepad1_LeftStickYValue;
    private float gamepad1_LeftStickXValue;
    private float gamepad1_TriggersValue;
    private double Straight;
    private double Strafe;
    private double Rotate;
    private double FastStraight;
    private double FastStrafe;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        initializeWheels();
        initializeDevices();
        initializePositions();

        waitForStart();
        if (opModeIsActive()) {
            // RUN BLOCKS:
            while (opModeIsActive()) {
                // LOOP BLOCKS:
                driveChassis();
                manageManipulatorControls();
            }
        }
    }

    private void manageManipulatorControls()
    {
        if (gamepad2.square) {
            extensionServoPosition = 0.1;
            Extension.setPosition(extensionServoPosition);
        } else if (gamepad2.triangle) {
            extensionServoPosition = 0.7;
            Extension.setPosition(extensionServoPosition);
        }

        if (gamepad2.circle) {
            deliveryServoPosition = 0.95;
            DeliveryBox.setPosition(deliveryServoPosition);
        } else if (gamepad2.cross) {
            deliveryServoPosition = 0.3;
            DeliveryBox.setPosition(deliveryServoPosition);
        }

        if (gamepad2.dpad_up) {
            IntakeBox.setPower(0.5);
        }
        else if(gamepad2.dpad_down) {
            IntakeBox.setPower(-0.5);
        }
        else {
            IntakeBox.setPower(0);
        }

        if(gamepad2.right_bumper){
            specimenServoPosition =0.8;
            SpecimenGripper.setPosition(specimenServoPosition);
        }

        if(gamepad2.left_bumper){
            specimenServoPosition =0;
            SpecimenGripper.setPosition(specimenServoPosition);
        }
    }

    private void initializePositions()
    {
        extensionServoPosition = 0.7;
        Extension.setPosition(extensionServoPosition);
        deliveryServoPosition = 0.3;
        DeliveryBox.setPosition(deliveryServoPosition);
        specimenServoPosition = 0;
        SpecimenGripper.setPosition(specimenServoPosition);

    }

    private void driveChassis()
    {
        gamepad1_RightStickYValue = -gamepad1.right_stick_y;
        gamepad1_RightStickXValue = gamepad1.right_stick_x;
        gamepad1_LeftStickYValue = -gamepad1.left_stick_y;
        gamepad1_LeftStickXValue = gamepad1.left_stick_x;
        gamepad1_TriggersValue = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1_RightStickYValue != 0 || gamepad1_RightStickXValue != 0 || gamepad1_LeftStickYValue != 0 || gamepad1_LeftStickXValue != 0 || gamepad1_TriggersValue != 0) {
            // Set robot's move forward(+) or backwards(-) power
            Straight = 0.5 * (0.75 * Math.pow(gamepad1_RightStickYValue, 3) + 0.25 * gamepad1_RightStickYValue);
            // Set robot's strafe right(+) or left(-) power
            Strafe = 0.5 * (0.75 * Math.pow(gamepad1_RightStickXValue, 3) + 0.25 * gamepad1_RightStickXValue);
            // Set robot's clockwise(+) or counter-clockwise(-) rotation power
            Rotate = -0.5 * (0.75 * Math.pow(gamepad1_TriggersValue, 3) + 0.25 * gamepad1_TriggersValue);
            // Set robot's fast move forward(+) or backwards(-) power
            FastStraight = 0.8 * gamepad1_LeftStickYValue;
            // Set robot's fast strafe right(+) or left(-) power
            FastStrafe = 0.8 * gamepad1_LeftStickXValue;
            // We switched the forward and back config on the wheels in init, so these are swapped from last year
            BackLeftWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) + Rotate);
            FrontRightWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) - Rotate);
            FrontLeftWheel.setPower(Straight + FastStraight + Strafe + FastStrafe + Rotate);
            BackRightWheel.setPower((Straight + FastStraight + Strafe + FastStrafe) - Rotate);
        } else {
            // Stop all motors if their controls are not touched
            BackLeftWheel.setPower(0);
            BackRightWheel.setPower(0);
            FrontLeftWheel.setPower(0);
            FrontRightWheel.setPower(0);
        }
    }

    private void initializeWheels()
    {
        BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
        FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");

        // INITIALIZATION BLOCKS:
        // > Reverse motors'/servos' direction as needed
        BackLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        BackRightWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        // > Set motors' ZeroPower behavior
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // > Clear Encoders of prior data
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeDevices()
    {
        Extension = hardwareMap.get(Servo.class, "Extension");
        DeliveryBox = hardwareMap.get(Servo.class, "DeliveryBox");
        IntakeBox = hardwareMap.get(CRServo.class, "IntakeBox");
        SpecimenGripper = hardwareMap.get(Servo.class,"SpecimenGripper");
    }
}
