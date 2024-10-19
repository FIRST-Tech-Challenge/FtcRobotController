package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.EventTracker;

@TeleOp
public class Teleop_Practice extends LinearOpMode {
    private int intakeExtensionMax = 3000;
    private int liftLowerBasket = 500;
    private int liftUpperBasket = 500;
    private int liftLowerSpecimenBar = 500;
    private int liftUpperSpecimenBar = 500;
    private int liftGetSpecimenFromWall = 500;
    private int liftHangOnLowerBar = 500;
    private int liftHangOnUpperBar = 1000;
    private double extensionServoHome = 0.7;
    private double extensionServoDump = 0.1;
    private double deliveryServoHome = 0.3;
    private double deliveryServoDump = 0.95;
    private double specimenServoOpen = 0;
    private double specimenServoClosed = 0.8;
    private double continuousIntakePower = 0.5;
    private DcMotor BackLeftWheel;
    private DcMotor FrontLeftWheel;
    private DcMotor BackRightWheel;
    private DcMotor FrontRightWheel;
    private DcMotor IntakeExtension;
    private int IntakeExtensionPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
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

    private ElapsedTime currentTimer;
    private EventTracker eventTracker;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        initializeWheels();
        initializeDevices();
        initializePositions();

        currentTimer = new ElapsedTime();
        eventTracker = new EventTracker();

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
            extensionServoPosition = extensionServoDump;
            Extension.setPosition(extensionServoPosition);
        } else if (gamepad2.triangle) {
            extensionServoPosition = extensionServoHome;
            Extension.setPosition(extensionServoPosition);
        }

        if (gamepad2.circle) {
            deliveryServoPosition = deliveryServoDump;
            DeliveryBox.setPosition(deliveryServoPosition);
        } else if (gamepad2.cross) {
            deliveryServoPosition = deliveryServoHome;
            DeliveryBox.setPosition(deliveryServoPosition);
        }

        if (gamepad2.right_trigger>0.4) {
            IntakeBox.setPower(continuousIntakePower);
        }
        else if(gamepad2.left_trigger>0.4) {
            IntakeBox.setPower(-specimenServoOpen);
        }
        else {
            IntakeBox.setPower(0);
        }

        if(gamepad2.right_bumper){
            specimenServoPosition =specimenServoClosed;
            SpecimenGripper.setPosition(specimenServoPosition);
        }

        if(gamepad2.left_bumper){
            specimenServoPosition =specimenServoOpen;
            SpecimenGripper.setPosition(specimenServoPosition);
        }
        if (gamepad2.ps) {
            Lift.setTargetPosition(0);
        } else if (gamepad2.dpad_down) {
            Lift.setTargetPosition(300);
        } else if (gamepad2.dpad_left) {
            Lift.setTargetPosition(600);
        } else if (gamepad2.dpad_up) {
            Lift.setTargetPosition(900);
        } else if (gamepad2.dpad_right) {
            Lift.setTargetPosition(1200);
        }

        while(-gamepad2.left_stick_y > 0.2)
        {
            if(eventTracker.getLastTimestamp("ExtendIntake") < currentTimer.seconds()-.15) {
                if (IntakeExtensionPosition < intakeExtensionMax) {
                    IntakeExtensionPosition += 100;
                    eventTracker.setTimestamp("ExtendIntake", currentTimer.seconds());
                    IntakeExtension.setTargetPosition(IntakeExtensionPosition);
                }
            }
        }
        while(-gamepad2.left_stick_y < -0.2)
        {
            if(eventTracker.getLastTimestamp("ExtendIntake") < currentTimer.seconds()-.15) {
                if (IntakeExtensionPosition > 100) {
                    IntakeExtensionPosition -= 100;
                    eventTracker.setTimestamp("ExtendIntake", currentTimer.seconds());
                    IntakeExtension.setTargetPosition(IntakeExtensionPosition);
                }
            }
        }

        /*
        if (gamepad2.ps) {
            IntakeExtension.setTargetPosition(0);
        } else if (gamepad2.dpad_down) {
            IntakeExtension.setTargetPosition(300);
        } else if (gamepad2.dpad_left) {
            IntakeExtension.setTargetPosition(600);
        } else if (gamepad2.dpad_up) {
            IntakeExtension.setTargetPosition(900);
        } else if (gamepad2.dpad_right) {
            IntakeExtension.setTargetPosition(1200);
        }
        */

    }

    private void initializePositions()
    {
        extensionServoPosition = extensionServoHome;
        Extension.setPosition(extensionServoPosition);
        deliveryServoPosition = deliveryServoHome;
        DeliveryBox.setPosition(deliveryServoPosition);
        specimenServoPosition = specimenServoOpen;
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
            Rotate = 0.5 * (0.75 * Math.pow(gamepad1_TriggersValue, 3) + 0.25 * gamepad1_TriggersValue);
            // Set robot's fast move forward(+) or backwards(-) power
            FastStraight = 0.8 * gamepad1_LeftStickYValue;
            // Set robot's fast strafe right(+) or left(-) power
            FastStrafe = 0.8 * gamepad1_LeftStickXValue;
            BackLeftWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) + Rotate);
            BackRightWheel.setPower((Straight + FastStraight + Strafe + FastStrafe) - Rotate);
            FrontLeftWheel.setPower(Straight + FastStraight + Strafe + FastStrafe + Rotate);
            FrontRightWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) - Rotate);
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
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackRightWheel.setDirection(DcMotor.Direction.FORWARD);
        FrontRightWheel.setDirection(DcMotor.Direction.FORWARD);
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
        IntakeExtension = hardwareMap.get(DcMotor.class, "IntakeExtension");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        ExtensionLimit = hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");


        IntakeExtension.setDirection(DcMotor.Direction.FORWARD);
        IntakeExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeExtension.setTargetPosition(0);
        IntakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeExtension.setPower(0.4);
        IntakeExtensionPosition = 0;
        // Initialize Lift
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.4);
    }
}
