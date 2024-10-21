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
    private int extensionSliderMax = 3000;
    private int liftLowerBasket = 900;
    private int liftUpperBasket = 2800;
    private int liftLowerSpecimenBar = 500;
    private int liftUpperSpecimenBar = 2000;
    private int liftSnapSpecimen = 200;
    private int liftGetSpecimenFromWall = 500;
    private int liftHangOnLowerBar = 2000;
    private int liftHangOnUpperBar = 1000;
    private double extensionServoHome = 0.72;
    private double extensionServoDump = 0.1;
    private double deliveryServoHome = 0.3;
    private double deliveryServoDump = 0.95;
    private double specimenServoOpen = 0;
    private double specimenServoClosed = 0.8;
    private double continuousIntakePower = 0.8;
    private DcMotor BackLeftWheel;
    private DcMotor FrontLeftWheel;
    private DcMotor BackRightWheel;
    private DcMotor FrontRightWheel;
    private DcMotor ExtensionSlider;
    private int extensionSliderPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
    private TouchSensor LiftLimit2;
    private Servo ExtensionServo;
    private Servo DeliveryBoxServo;
    private CRServo IntakeBoxServo;
    public Servo SpecimenGripperServo;
    private double extensionServoPosition;
    private double extensionServoSafetyPosition = 0.5;
    private double deliveryBoxServoPosition;
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
    private double highSpeedDrive = 0.7;
    private double lowSpeedDrive = 0.4;
    private int liftGoToPosition = 0;

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
                manageDriverControls();
                manageManipulatorControls();

                if(eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.3))
                {
                    telemetry.update();
                }
            }
        }
    }

    private void manageDriverControls()
    {
        if(gamepad1.square)
        {
            resetExtensionSlider();
        }
        if(gamepad1.circle)
        {
            resetLift();
        }
    }
    private void manageManipulatorControls()
    {
        if (gamepad2.square) {
            if(checkIsLiftDown() && checkIsExtensionHome()) {
                extensionServoPosition = extensionServoDump;
                ExtensionServo.setPosition(extensionServoPosition);
            }
        } else if (gamepad2.triangle) {
            extensionServoPosition = extensionServoHome;
            ExtensionServo.setPosition(extensionServoPosition);
        }

        if (gamepad2.circle) {
            checkExtensionServoSafety();
            deliveryBoxServoPosition = deliveryServoDump;
            DeliveryBoxServo.setPosition(deliveryBoxServoPosition);
        } else if (gamepad2.cross) {
            checkExtensionServoSafety();
            deliveryBoxServoPosition = deliveryServoHome;
            DeliveryBoxServo.setPosition(deliveryBoxServoPosition);
        }

        if (gamepad2.right_trigger>0.4) {
            IntakeBoxServo.setPower(continuousIntakePower);
        }
        else if(gamepad2.left_trigger>0.4) {
            IntakeBoxServo.setPower(-continuousIntakePower);
        }
        else {
            IntakeBoxServo.setPower(0);
        }

        if(gamepad2.right_bumper){
            specimenServoPosition =specimenServoClosed;
            SpecimenGripperServo.setPosition(specimenServoPosition);
        }

        if(gamepad2.left_bumper){
            specimenServoPosition =specimenServoOpen;
            SpecimenGripperServo.setPosition(specimenServoPosition);
        }
        if (gamepad2.dpad_down) {
            checkExtensionServoSafety();
            deliveryBoxServoPosition = deliveryServoHome;
            DeliveryBoxServo.setPosition(deliveryBoxServoPosition);
            Lift.setTargetPosition(30);
        } else if (gamepad2.dpad_left) {
            checkExtensionServoSafety();
            Lift.setTargetPosition(liftLowerBasket);
        } else if (gamepad2.dpad_up) {
            checkExtensionServoSafety();
            Lift.setTargetPosition(liftUpperBasket);
        } else if (gamepad2.dpad_right) {
            checkExtensionServoSafety();
            Lift.setTargetPosition(liftUpperSpecimenBar);
        }

        if(-gamepad2.left_stick_y > 0.2)
        {
            if(eventTracker.doEvent("ExtendIntake", currentTimer.seconds(), 0.10))
            {
                if (extensionSliderPosition < extensionSliderMax) {
                    ExtensionServo.setPosition(0.6);
                    extensionSliderPosition += 100;
                    ExtensionSlider.setTargetPosition(extensionSliderPosition);
                }
            }
        }
        if(-gamepad2.left_stick_y < -0.2)
        {
            if(eventTracker.doEvent("ExtendIntake", currentTimer.seconds(), 0.10)) {
                if (extensionSliderPosition > 100) {
                    ExtensionServo.setPosition(0.6);
                    extensionSliderPosition -= 100;
                    if(extensionSliderPosition < 150)
                    {
                        resetExtensionSlider(1000);
                    }
                    else {
                        ExtensionSlider.setTargetPosition(extensionSliderPosition);
                    }
                }
            }
        }



    }

    private void initializePositions()
    {
        extensionServoPosition = extensionServoHome;
        ExtensionServo.setPosition(extensionServoPosition);
        deliveryBoxServoPosition = deliveryServoHome;
        DeliveryBoxServo.setPosition(deliveryServoHome);
        specimenServoPosition = specimenServoOpen;
        SpecimenGripperServo.setPosition(specimenServoPosition);

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
            Straight = lowSpeedDrive * (0.75 * Math.pow(gamepad1_RightStickYValue, 3) + 0.25 * gamepad1_RightStickYValue);
            // Set robot's strafe right(+) or left(-) power
            Strafe = lowSpeedDrive * (0.75 * Math.pow(gamepad1_RightStickXValue, 3) + 0.25 * gamepad1_RightStickXValue);
            // Set robot's clockwise(+) or counter-clockwise(-) rotation power
            Rotate = lowSpeedDrive * (0.75 * Math.pow(gamepad1_TriggersValue, 3) + 0.25 * gamepad1_TriggersValue);
            // Set robot's fast move forward(+) or backwards(-) power
            FastStraight = highSpeedDrive * gamepad1_LeftStickYValue;
            // Set robot's fast strafe right(+) or left(-) power
            FastStrafe = highSpeedDrive * gamepad1_LeftStickXValue;
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
        ExtensionServo = hardwareMap.get(Servo.class, "Extension");
        DeliveryBoxServo = hardwareMap.get(Servo.class, "DeliveryBox");
        IntakeBoxServo = hardwareMap.get(CRServo.class, "IntakeBox");
        SpecimenGripperServo = hardwareMap.get(Servo.class,"SpecimenGripper");
        ExtensionSlider = hardwareMap.get(DcMotor.class, "IntakeExtension");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        ExtensionLimit = hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");
        LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit2");

        resetExtensionSlider(3000);

        resetLift();
    }

    private void resetExtensionSlider(long safetyDuration)
    {

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = safetyDuration; // Maximum duration in milliseconds (3 seconds)

        while(!ExtensionLimit.isPressed()) {
            ExtensionSlider.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        ExtensionSlider.setPower(0);

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionSlider.setTargetPosition(0);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionSlider.setPower(0.4);
        extensionSliderPosition = 0;
    }

    private void resetLift()
    {

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = 3000; // Maximum duration in milliseconds (3 seconds)
        while(!LiftLimit.isPressed() && !LiftLimit2.isPressed()) {
            Lift.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        Lift.setPower(0);

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.6);
    }

    private void checkExtensionServoSafety()
    {
        if(ExtensionServo.getPosition() < extensionServoSafetyPosition)
        {
            ExtensionServo.setPosition(extensionServoSafetyPosition);
        }
    }

    private boolean checkIsLiftDown()
    {
        return (Lift.getCurrentPosition() < 150 && DeliveryBoxServo.getPosition() == deliveryServoHome);
    }
    private boolean checkIsExtensionHome()
    {
        return (ExtensionSlider.getCurrentPosition() < 100);
    }
}
