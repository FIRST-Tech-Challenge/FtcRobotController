package org.firstinspires.ftc.teamcode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MainOp extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        //Initialization

        //Drivechain Motors
        DcMotor wheelFrontLeft = hardwareMap.dcMotor.get("wheelFrontLeft");
        DcMotor wheelFrontRight = hardwareMap.dcMotor.get("wheelFrontRight");
        DcMotor wheelBackLeft = hardwareMap.dcMotor.get("wheelBackLeft");
        DcMotor wheelBackRight = hardwareMap.dcMotor.get("wheelBackRight");

        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE); //CHECK MOTORS FOR WHICH ONES ARE REVERSED
        wheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Main lin slide Motors
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");

        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Secondary, single motor lin slide used to hoist the robot at the end of the game
        //DcMotor liftingLinSlide = hardwareMap.dcMotor.get("liftingLinSlide");

        //liftingLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //liftingLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftingLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake (GoGekko Wheel) Continuous Servo
        CRServo goGekkoCRServo = hardwareMap.crservo.get("goGekkoCRServo");

        //Intake (Hinging) Servo
        CRServo hingeCRServo = hardwareMap.crservo.get("hingeCRServo");

        //Intake (Hatch) Servo
        Servo hatchServo = hardwareMap.servo.get("hatchServo");

        //Running initialization methods
        Drivechain.driveChainInit(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
        IntakeGekkoWheelCRServo.gekkoWheelInit(goGekkoCRServo);
        MainLinearSlides.linearSlideInit(leftLinSlide, rightLinSlide);
        //LiftingSlide.liftingSlideInit(liftingLinSlide); //ADD BACK ONCE WRITTEN
        IntakeHingeCRServo.intakeHingeServoInit(hingeCRServo);
        IntakeHatchServo.hatchServoInit(hatchServo);


        //OpMode
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {

            //Controller Joystick Initialization
            double yLeftStickCTLR1 = -gamepad1.left_stick_y;
            double xLeftStickCTLR1 = gamepad1.left_stick_x;
            double yRightStickCTLR1 = -gamepad1.right_stick_y;
            double xRightStickCTLR1 = gamepad1.right_stick_x;

            double yLeftStickCTLR2 = gamepad2.left_stick_y;
            double xLeftStickCTLR2 = gamepad2.left_stick_x;
            double yRightStickCTLR2 = gamepad2.right_stick_y;
            double xRightStickCTLR2 = gamepad2.right_stick_x;

            //Controller Button Initialization
            boolean xButtonCTLR1 = gamepad1.x;
            boolean yButtonCTLR1 = gamepad1.y;
            boolean aButtonCTLR1 = gamepad1.a;
            boolean bButtonCTLR1 = gamepad1.b;

            boolean xButtonCTLR2 = gamepad2.x;
            boolean yButtonCTLR2 = gamepad2.y;
            boolean aButtonCTLR2 = gamepad2.a;
            boolean bButtonCTLR2 = gamepad2.b;

            //Controller Dpad Initialization
            boolean dpadUpCTRL1 = gamepad1.dpad_up;
            boolean dpadDownCTRL1 = gamepad1.dpad_down;
            boolean dpadLeftCTRL1 = gamepad1.dpad_left;
            boolean dpadRightCTRL1 = gamepad1.dpad_right;

            boolean dpadUpCTRL2 = gamepad2.dpad_up;
            boolean dpadDownCTRL2 = gamepad2.dpad_down;
            boolean dpadLeftCTRL2 = gamepad2.dpad_left;
            boolean dpadRightCTRL2 = gamepad2.dpad_right;

            //Controller Bumper Initialization
            boolean leftBumperCTRL1 = gamepad1.left_bumper;
            boolean rightBumperCTRL1 = gamepad1.right_bumper;

            boolean leftBumperCTRL2 = gamepad2.left_bumper;
            boolean rightBumperCTRL2 = gamepad2.right_bumper;

            //Controller Trigger Intialization
            float leftTriggerCTRL1 = gamepad1.left_trigger;
            float rightTriggerCTRL1 = gamepad1.right_trigger;

            float leftTriggerCTRL2 = gamepad2.left_trigger;
            float rightTriggerCTRL2 = gamepad2.right_trigger;

            Drivechain.motorDirectionTest(yLeftStickCTLR1, xRightStickCTLR1); //ON TESTING METHOD
            MainLinearSlides.manualMove(leftTriggerCTRL1, rightTriggerCTRL1);
            IntakeGekkoWheelCRServo.runWheels(dpadUpCTRL1);
            MainLinearSlides.moveToLowerUpper(leftBumperCTRL1, rightBumperCTRL1);
            IntakeHingeCRServo.manualHinge(aButtonCTLR1, bButtonCTLR1);
            IntakeHatchServo.hinge(dpadDownCTRL1);
        }
    }
}
