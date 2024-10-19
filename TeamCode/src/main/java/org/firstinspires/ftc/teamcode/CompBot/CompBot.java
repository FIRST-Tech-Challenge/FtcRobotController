package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {

    SwerveConfig swerve = new SwerveConfig(this);

    GoBildaPinpointDriver odo;

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide;

    Servo FLServo, BLServo, BRServo, FRServo, claw;


    // In case builders are bad, is offset center for servo
    double FLServoOffSet = .00;     //0.00
    double FRServoOffSet = .00;     //0.00
    double BLServoOffSet = .00;     //0.01
    double BRServoOffSet = .00;     //.007


    /**
     * controls for game pad 1:
     * right trigger: forwards
     * left trigger: backwards
     * right stick x: rotate
     * left stick x: strafe
     * <p>
     * controls for game pad 2:
     * left stick y: in and out of arm
     * right stick y: up and down of arm
     * left trigger: claw intake
     * right trigger: claw out
     * presets for:
     * attaching clip to sample
     * attaching specimen(clip + sample) to top rung
     * presets for bucket 1 and 2
     */
    public void runOpMode() throws InterruptedException {

        swerve.initSwerve(); // Does all the robot stuff

        initRobot();


        waitForStart();
        while (opModeIsActive()) {

            //game pad 1
            double speedGMP1 = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angleGMP1 = -gamepad1.right_stick_x;

            if (speedGMP1 != 0) swerve.moveStraight(gamepad1.left_stick_x, speedGMP1);
            else if (angleGMP1 != 0) swerve.rotate(angleGMP1);
            else {
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }
            //if (gamepad1.a) rotateToCenter();


            //game pad 2 inputs
            double armLength = -gamepad2.right_stick_y;
            double armAngle = -gamepad2.left_stick_y;

            //call move arm
            extendArm(armLength);
            liftArm(armAngle);

            //to test arm length and angle
            addTelem(pivot.getCurrentPosition(), pivot.getCurrentPosition(), armAngle, armLength);
        }
    }


    // to lift arm, input from game pad 2 straight in
    public void liftArm(double input) {

        if (input > 0 && pivot.getCurrentPosition() < 2904) {
            pivot.setTargetPosition(2905);
        } else if (input < 0 && pivot.getCurrentPosition() > -2904) {
            input = 0;
            pivot.setTargetPosition(-2905);
        }

        pivot.setPower(input);
    }

    // to extend arm, input from game pad 2 straight in
    public void extendArm(double input) {

        if (input > 0.01 && slide.getCurrentPosition() > 4268) {
            input = 0;
            slide.setTargetPosition(4268);
        } else if (input < -0.01 && slide.getCurrentPosition() < -4268) {
            input = 0;
            slide.setTargetPosition(-4268);
        }

        slide.setPower(input);
    }


    // to test arm lift and extend
    public void addTelem(int x, int y, double a, double b) {
        telemetry.addData("Arm angle: ", x);
        telemetry.addData("Arm length: ", y);
        telemetry.addData("Left stick y: ", a);
        telemetry.addData("Right stick y: ", b);
        telemetry.addData("Pivot target pos: ", pivot.getTargetPosition());
        telemetry.addData("Arm target pos: ", slide.getTargetPosition());
        telemetry.addData("Pivot power: ", pivot.getPower());
        telemetry.addData("Arm power: ", slide.getPower());
        telemetry.addData("Pivot position: ", pivot.getCurrentPosition());
        telemetry.addData("Arm position: ", slide.getCurrentPosition());
        telemetry.update();
    }


    /**
     * Initializes the robot.<br>
     * Starts all the devices and maps where they go
     * As well as sets direction and whether motors run with encoders or not
     */
    public void initRobot() {

        // Maps the motor objects to the physical ports
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Sets the encoder mode
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);

        FLMotor.setDirection(REVERSE);
        BLMotor.setDirection(REVERSE);
        FRMotor.setDirection(REVERSE);
        BRMotor.setDirection(FORWARD);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(FLServoOffSet, 1.0 + FLServoOffSet * 2);
        BLServo.scaleRange(BLServoOffSet, 1.0 + BLServoOffSet * 2);
        FRServo.scaleRange(FRServoOffSet, 1.0 + FRServoOffSet * 2);
        BRServo.scaleRange(BRServoOffSet, 1.0 + BRServoOffSet * 2);

        FLServo.setPosition(0.50 + FLServoOffSet);
        BLServo.setPosition(0.50 + BLServoOffSet);
        FRServo.setPosition(0.50 + FRServoOffSet);
        BRServo.setPosition(0.50 + BRServoOffSet);


        // Init GoBilda Pinpoint module
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(177.8, 50.8);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        // Init slaw, claw, and pivot
        pivot = hardwareMap.dcMotor.get("pivot");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");

        pivot.setMode(RUN_USING_ENCODER);
        slide.setMode(RUN_USING_ENCODER);
        claw.scaleRange(0, 1);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        // Sets pivot and slide to pos 0
        pivot.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(STOP_AND_RESET_ENCODER);

        pivot.setPower(0);
        slide.setPower(0);
    }


}
