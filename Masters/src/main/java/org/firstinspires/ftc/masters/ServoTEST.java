package org.firstinspires.ftc.masters;


import static org.firstinspires.ftc.masters.CSCons.clawOpen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons.ClawPosition;
import org.firstinspires.ftc.masters.CSCons.DriveMode;
import org.firstinspires.ftc.masters.CSCons.HookPosition;
import org.firstinspires.ftc.masters.CSCons.IntakeState;
import org.firstinspires.ftc.masters.CSCons.OuttakePosition;
import org.firstinspires.ftc.masters.CSCons.OuttakeState;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "SERVO TEST", group = "test")
public class ServoTEST extends LinearOpMode {

    static int target = 0;
    OuttakePosition backSlidePos = OuttakePosition.BOTTOM;

    private final double ticks_in_degrees = 384.5 / 180;

    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

//    double y = 0;
//    double x = 0;
//    double rx = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor intakeSlides = null;
    DcMotor backSlides = null;
    DcMotor otherBackSlides = null;
    DcMotor hangingMotor = null;

    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo microHook;

    TouchSensor touchBucket;

    boolean clawClosed = true;
    boolean hookEngaged = false;
    boolean hookPressed = false;
    boolean presetPushed = false;
    double claw2transfer_time = -1;
    double claw2transfer_delay = .5;
    double x_last_pressed = -1;


    int backSlidesTargetPos = 0;
    int presetBackSlidesTargetPos = 0;
    double bucketMovedBy = 0;
    boolean outtakeGoingToTransfer;
    double outtakeRotationTarget;

    int v4bPresetTarget = 0;
    private double claw_last_opened;

    private enum Retract {
        back,
        flip_bar,
        cartridge
    }

    private boolean isRetracting = false;

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.ReadyToTransfer;
    private IntakeState intakeState = IntakeState.Transition;

    private Retract retract = Retract.back;

    private ClawPosition clawPosition = ClawPosition.OPEN;
    private HookPosition hookPosition = HookPosition.OPEN;
    RevColorSensorV3 colorSensor;
    double angleRotationAdjustment = 0;

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
    TODO: D-pad controls extendo
    TODO: D-pad up - extendo fully extended with four bar in position 4
    TODO: D-pad right - extendo out 2/3? with four bar in position 1
    TODO: D-pad down - extendo out 1/3? with four bar in position 1
    TODO: D-pad left - extendo fully in and four bar in position 1
    Buttons
    TODO: A - claw opens and closes
    TODO: B - transfers and moves to slide pos 1
    TODO: X - auto aligns and switches to pixel scoring mode
    TODO: Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
    TODO: LB - four bar down (presets)
    TODO: RB - four bar up (presets)
    TODO: LT - slides down (presets)
    TODO: RT - slides up (presets)

    Pixel scoring mode - entered with x while in normal
    Only moves right and left no forward/ backwards
    TODO: LT - slides down (presets)
    TODO: RT - slides up (presets)
    TODO: Y - press once drop one pixel, hold for drop both pixels

    Endgame mode - entered with right stick down
    Normal driving
    TODO: X - Auto aligns on April tag for shooter, raise shooter, shoot
    TODO: LB - Hang down
    TODO: RB - Hang up
        */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");
        backSlides = hardwareMap.dcMotor.get("backSlides");
        otherBackSlides = hardwareMap.dcMotor.get("otherBackSlides");

        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        //cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        microHook = hardwareMap.servo.get("microHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovement = hardwareMap.servo.get("backSlideServo");
        touchBucket = hardwareMap.touchSensor.get("touchBucket");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");


        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        otherBackSlides.setDirection(DcMotorSimple.Direction.REVERSE);

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
        otherBackSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlides.setPower(.5);

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawArm.setPosition(CSCons.clawArmTransition);
        clawAngle.setPosition(CSCons.clawAngleTransition);
        clawServo.setPosition(CSCons.clawOpen);

        outtakeMovement.setPosition(CSCons.outtakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
        outtakeHook.setPosition(CSCons.openHook);
        microHook.setPosition(CSCons.openMicroHook);
        hookPosition = HookPosition.OPEN;
        planeRaise.setPosition(CSCons.droneFlat);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        target = backSlidePos.getTarget();
        angleRotationAdjustment = 0;

        ElapsedTime intakeElapsedTime = null, outtakeElapsedTime = null;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean buttonPushed = false;

        waitForStart();

        runtime.reset();
        ElapsedTime elapsedTime;
        ElapsedTime colorSensorElapsedTime = null;
        ElapsedTime flipElapsedTime = null;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());




                if (gamepad2.a) {
                    telemetry.addData("a pushed", gamepad2.a);
                    clawArm.setPosition(CSCons.clawArmTransfer);
                }
                if (gamepad2.b) {
                    telemetry.addData("b pushed", gamepad2.b);
                    clawArm.setPosition(CSCons.clawArmGround);
                }


//            if (gamepad1.a ) {
//                clawPosition = ClawPosition.TRANSFER;
//                clawServo.setPosition(CSCons.clawTransfer);
//            }

//                telemetry.addData("left y", gamepad1.left_stick_y);
//                telemetry.addData("left x", gamepad1.left_stick_x);
//                telemetry.addData("right x", gamepad1.right_stick_x);
//                telemetry.addData("Arm", clawArm.getPosition());
//                telemetry.addData("backSlides", backSlides.getCurrentPosition());
//                telemetry.addData("intakeSides", intakeSlides.getCurrentPosition());// 1725, 2400,
//                telemetry.addData("time", runtime.time());
//                telemetry.addData("Status", colorSensor.status());
//                telemetry.addData("Outtake state", outtakeState.name());
//                telemetry.addData("Intake state", intakeState.name());
//                telemetry.addData("back left", leftRearMotor.getCurrentPosition());
//                telemetry.addData("back right", rightRearMotor.getCurrentPosition());
//                telemetry.addData("front left", leftFrontMotor.getCurrentPosition());
//                telemetry.addData("front right", rightFrontMotor.getCurrentPosition());
//                telemetry.addData("TOUCH:", touchBucket.isPressed());
//                telemetry.update();

                telemetry.update();
            }



    }

    protected void drive(double x, double y, double rx) {

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - (x*.69) + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + (x*.69) - rx;

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
    }

    protected void backSlidesMove(int target) {

        int slidePos = backSlides.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double liftPower = pid + ff;

        backSlides.setPower(liftPower);
        otherBackSlides.setPower(liftPower);
    }

    protected ElapsedTime closeHook(){
        ElapsedTime time = new ElapsedTime();
        hookPosition = HookPosition.CLOSED;
        outtakeHook.setPosition(CSCons.closeHook);
        microHook.setPosition(CSCons.closeMicroHook);
        return time;
    }

    protected boolean detectPixel(){
        if (colorSensor.getRawLightDetected() > CSCons.pixelDetectThreshold && runtime.time(TimeUnit.MILLISECONDS) > claw_last_opened + 1000){
            return true;
        } else {
            return false;
        }
    }

}
