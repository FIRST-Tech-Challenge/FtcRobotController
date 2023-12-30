package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name="Center Stage TEST", group = "competition")
public class CenterStageTest extends LinearOpMode {

    static int target = 0;
    static int backSlidePos = 0;

    private final double ticks_in_degrees = 384.5 / 180;

    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

    double y = 0;
    double x = 0;
    double rx = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor gpSlideRight = null;
    DcMotor gpSlideLeft = null;
    DcMotor backSlides = null;
    DcMotor hangingMotor = null;

    Servo planeLaunch;
    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovementRight;
    Servo outtakeMovementLeft;

    TouchSensor touchSensor;

    boolean clawClosed = true;
    boolean hookEngaged = false;
    boolean hookPressed = false;
    boolean presetPushed = false;
    double claw2transfer_time = -1;
    double claw2transfer_delay=.5;
    double x_last_pressed=-1;

    int backSlidesTargetPos = 0;
    int presetBackSlidesTargetPos = 0;
    boolean outtakeGoingToTransfer;
    double outtakeRotationTarget;

    int v4bPresetTarget = 0;

    private enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME
    }

    private enum Retract {
        back,
        flip_bar,
        cartridge
    }

    private boolean isRetracting = false;

    private DriveMode driveMode = DriveMode.NORMAL;
    private Retract retract = Retract.back;
    @Override
    public void runOpMode() {
        /*
            Controls
            --------
            Gamepad2

            Gamepad1

            --------
Drive with sticks intake is the front
Four bar position 1-5 auto stack pixels
Slides position 1-11 placement on backboard
Normal mode - pixel grabbing mode - entered with left stick down
D-pad controls extendo
D-pad up - extendo fully extended with four bar in position 4
D-pad right - extendo out 2/3? with four bar in position 1
D-pad down - extendo out 1/3? with four bar in position 1
D-pad left - extendo fully in and four bar in position 1
Buttons
A - claw opens and closes
B - transfers and moves to slide pos 1
X - auto aligns and switches to pixel scoring mode
Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
LB - four bar down (presets)
RB - four bar up (presets)
LT - slides down (presets)
RT - slides up (presets)

Pixel scoring mode - entered with x while in normal
Only moves right and left no forward/ backwards
LT - slides down (presets)
RT - slides up (presets)
Y - press once drop one pixel, hold for drop both pixels

Endgame mode - entered with right stick down
Normal driving
X - Auto aligns on April tag for shooter, raise shooter, shoot
LB - Hang down
RB - Hang up
        */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        gpSlideLeft = hardwareMap.dcMotor.get("gpSlideLeft");
        gpSlideRight = hardwareMap.dcMotor.get("gpSlideRight");
        backSlides = hardwareMap.dcMotor.get("backSlides");

        planeLaunch = hardwareMap.servo.get("planeLaunch");
        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovementRight = hardwareMap.servo.get("outtakeMovementRight");
        outtakeMovementLeft = hardwareMap.servo.get("outtakeMovementLeft");
        touchSensor = hardwareMap.touchSensor.get("touch");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //backSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        clawArm.setPosition(CSCons.clawArmTransition);
        clawAngle.setPosition(CSCons.clawAngleTransition);
        clawServo.setPosition(CSCons.claw[2]);

        outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeMovementRight.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
        outtakeHook.setPosition(CSCons.outtakeHook[0]);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());

            int SlidePos = backSlides.getCurrentPosition();
            double pid = controller.calculate(SlidePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double liftPower = pid + ff;

            backSlides.setPower(liftPower);

            if (backSlidePos == 0) {
                target = 30;
            } else if (backSlidePos == 1) {
                target = 1800;
            } else if (backSlidePos == 2) {
                target = 2600;
            }

            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (gamepad2.b && clawClosed == true) {
                clawClosed = false;
                clawServo.setPosition(CSCons.claw[0]);
            } else if (gamepad2.a && clawClosed == false) {
                clawClosed = true;
                clawServo.setPosition(CSCons.claw[1]);
            }

            if (gamepad2.dpad_down) {
                clawAngle.setPosition(CSCons.clawAngleGroundToThree);
                clawArm.setPosition(CSCons.clawArmG);
            }

            if (gamepad2.dpad_up) {
                    clawAngle.setPosition(CSCons.clawAngleTransfer);
                    clawArm.setPosition(CSCons.clawArmTransfer+CSCons.skewampus);
            }

            if (gamepad2.dpad_right) {
                clawAngle.setPosition(CSCons.clawAngleTransfer);
                clawArm.setPosition(CSCons.clawArmTransfer);
            }

            if (gamepad2.dpad_left){
                clawAngle.setPosition(CSCons.clawAngleTransition);
                clawArm.setPosition(CSCons.clawArmTransition);
            }

            if (gamepad2.x && outtakeHook.getPosition() <= CSCons.outtakeHook[1] +.1 && outtakeHook.getPosition() >= CSCons.outtakeHook[1]-.1 && runtime.time() > x_last_pressed + .4) {
                outtakeHook.setPosition(CSCons.outtakeHook[0]);
                x_last_pressed = runtime.time();
            } else if (gamepad2.x && outtakeHook.getPosition() <= CSCons.outtakeHook[0] +.1 && outtakeHook.getPosition() >= CSCons.outtakeHook[0]-.1 && runtime.time() > x_last_pressed + .4) {
                outtakeHook.setPosition(CSCons.outtakeHook[1]);
                x_last_pressed = runtime.time();
            }

            if (gamepad2.left_stick_y > 0.2) {
                outtakeHook.setPosition(CSCons.outtakeHook[1]);
                outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackDrop);
                outtakeMovementRight.setPosition(CSCons.outtakeMovementBackDrop);
                outtakeRotationTarget = CSCons.outtakeAngleFolder;
            }
            if (gamepad2.left_stick_y < -0.2 && backSlidePos != 0) {
                outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackTransfer);
                outtakeMovementRight.setPosition(CSCons.outtakeMovementBackTransfer);
                outtakeGoingToTransfer = true;
                outtakeRotationTarget = CSCons.outtakeAngleTransfer;
                outtakeHook.setPosition(CSCons.outtakeHook[0]);
            }

            if (gamepad2.left_bumper) {
                outtakeRotationTarget += .005;
            }

            outtakeRotation.setPosition(outtakeRotationTarget);


            if (gamepad2.left_trigger > 0.5) {
                backSlidePos = 1;
            }
            if (gamepad2.right_trigger > 0.5) {
                backSlidePos = 2;
            }



            if (gamepad2.right_bumper){
                backSlidePos = 0;
            }

            if (gamepad1.a){
                clawServo.setPosition(CSCons.claw[2]);
            }

                    if (Math.abs(y) < 0.2) {
                        y = 0;
                    }
                    if (Math.abs(x) < 0.2) {
                        x = 0;
                    }

                    double leftFrontPower = y + x + rx;
                    double leftRearPower = y - x + rx;
                    double rightFrontPower = y - x - rx;
                    double rightRearPower = y + x - rx;

                    if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                        double max;
                        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                        max = Math.max(max, Math.abs(rightFrontPower));
                        max = Math.max(max, Math.abs(rightRearPower));

                        leftFrontPower /= max;
                        leftRearPower /= max;
                        rightFrontPower /= max;
                        rightRearPower /= max;
                    }

                    leftFrontMotor.setPower(leftFrontPower);
                    leftRearMotor.setPower(leftRearPower);
                    rightFrontMotor.setPower(rightFrontPower);
                    rightRearMotor.setPower(rightRearPower);

                    telemetry.addData("Arm", clawArm.getPosition());
                    telemetry.addData("backSlides",backSlidesTargetPos); // 1725, 2400,
                    telemetry.addData("time",runtime.time());

                    telemetry.update();
            }
        }
    }
