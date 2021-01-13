package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

public class UltimateBot extends YellowBot {
    public DcMotor wobbleSwing = null;
    private Servo wobbleClaw1 = null;
    private Servo wobbleClaw2 = null;
    private Servo ringCamera = null;
    private Servo shooterServo = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;

    private SwingPosition swingPosition = SwingPosition.Init;
    private static int SWING_GROUND_POS = 260;
    private static int SWING_LIFT_UP_POS = 160;
    private static int SWING_LIFT_WALL = 50;
    private static int STRAIGHT_UP = 70;
    private static double SHOOT_SERVO = 0.5;

    private static double CAMERA_RIGHT_LINE = 0.35;
    private static double CAMERA_LEFT_LINE = 0.5;

    private RingDetector rf = null;


    /* Constructor */
    public UltimateBot() {

    }

    @Override
    public void init(LinearOpMode owner, HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        super.init(owner, ahwMap, telemetry);

        try {
            wobbleSwing = hwMap.get(DcMotor.class, "swing");
            wobbleSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleSwing.setDirection(DcMotor.Direction.FORWARD);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleSwing.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleSwing. Check the controller config", ex);
        }

        try {
            intake = hwMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with intake. Check the controller config", ex);
        }

        try {
            shooter = hwMap.get(DcMotor.class, "shooter");
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with shooter. Check the controller config", ex);
        }

        try {
            wobbleClaw1 = hwMap.get(Servo.class, "claw1");
            wobbleClaw1.setPosition(1);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw1. Check the controller config", ex);
        }

        try {
            wobbleClaw2 = hwMap.get(Servo.class, "claw2");
            wobbleClaw2.setPosition(0);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw2. Check the controller config", ex);
        }

        try {
            ringCamera = hwMap.get(Servo.class, "camera");
            ringCamera.setPosition(CAMERA_RIGHT_LINE);
        } catch (Exception ex) {
            throw new Exception("Issues with ringCamera. Check the controller config", ex);
        }

        try {
            shooterServo = hwMap.get(Servo.class, "shoot");
            shooterServo.setPosition(SHOOT_SERVO);
        } catch (Exception ex) {
            throw new Exception("Issues with shooterServo. Check the controller config", ex);
        }


        telemetry.addData("Init", "Ultimate is ready");
    }


    public int moveWobbleSwing(double speed) {
        double power = Range.clip(speed, -1.0, 1.0);

        wobbleSwing.setPower(power);
        int position = wobbleSwing.getCurrentPosition();
        return position;
    }


    public void moveRingCamera(double position) {
        double p = Range.clip(position, -1.0, 1.0);
        ringCamera.setPosition(p);
    }

    @BotAction(displayName = "Move Intake", defaultReturn = "")
    public void intake() {
        if (intake != null) {
            intake.setPower(0.9);
        }
    }

    @BotAction(displayName = "Move Intake Reverse", defaultReturn = "")
    public void intakeReverse() {
        if (intake != null) {
            intake.setPower(-0.7);
        }
    }

    @BotAction(displayName = "Stop Intake", defaultReturn = "")
    public void stopintake() {
        if (intake != null) {
            intake.setPower(0);
        }
    }

    @BotAction(displayName = "Move Shooter", defaultReturn = "")
    public void shooter() {
        if (shooter != null) {
            shooter.setPower(0.9);
        }
    }

    @BotAction(displayName = "Move Peg Shooter", defaultReturn = "")
    public void shooterpeg() {
        if (shooter != null) {
            shooter.setPower(0.8);
        }
    }

    @BotAction(displayName = "Shoot", defaultReturn = "")
    public void shootServo() {
        ElapsedTime runtime = new ElapsedTime();
        if (shooterServo != null) {
            shooterServo.setPosition(0.1);
            runtime.reset();
            while (runtime.milliseconds() <= 250) {

            }
            shooterServo.setPosition(SHOOT_SERVO);
        }
    }

    @BotAction(displayName = "Stop Shooter", defaultReturn = "")
    public void stopshooter() {
        if (shooter != null) {
            shooter.setPower(0);
        }
    }

    @BotAction(displayName = "Close Claw", defaultReturn = "")
    public void closeWobbleClaw() {
        if ((wobbleClaw1 != null) && (wobbleClaw2 != null)) {
            wobbleClaw1.setPosition(0);
            wobbleClaw2.setPosition(1);
        }

    }

    @BotAction(displayName = "Open Claw", defaultReturn = "")
    public void openWobbleClaw() {
        if ((wobbleClaw1 != null) && (wobbleClaw2 != null)) {
            wobbleClaw1.setPosition(1);
            wobbleClaw2.setPosition(0);
        }
    }

    @BotAction(displayName = "Camera Left", defaultReturn = "")
    public void leftRingCamera() {
        if (ringCamera != null) {
            ringCamera.setPosition(CAMERA_LEFT_LINE);
        }
    }

    @BotAction(displayName = "Camera Right", defaultReturn = "")
    public void rightRingCamera() {
        if (ringCamera != null) {
            ringCamera.setPosition(CAMERA_RIGHT_LINE);
        }
    }

    @BotAction(displayName = "Init WobbleSwing", defaultReturn = "")
    public void backWobbleSwing() {
        if (wobbleSwing != null) {
            if (this.getSwingPosition() == SwingPosition.Init) {
                return;
            }
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setTargetPosition(0);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleSwing.setPower(0.6);
            boolean stop = false;
            while (!stop) {
                stop = wobbleSwing.isBusy() == false;
            }
            this.swingPosition = SwingPosition.Init;
            wobbleSwing.setPower(0);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @BotAction(displayName = "Place Wobble", defaultReturn = "")
    public void forwardWobbleSwing() {
        if (wobbleSwing != null) {
            if (this.getSwingPosition() == SwingPosition.Ground) {
                return;
            }
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setTargetPosition(SWING_GROUND_POS);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleSwing.setPower(0.6);
            boolean stop = false;
            while (!stop) {
                stop = wobbleSwing.isBusy() == false;
            }
            this.swingPosition = SwingPosition.Ground;
            wobbleSwing.setPower(0);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @BotAction(displayName = "Lift Wobble Up", defaultReturn = "")
    public void liftAndHoldWobbleSwing() {
        if (wobbleSwing != null) {
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setTargetPosition(SWING_LIFT_UP_POS);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleSwing.setPower(0.7);
            boolean stop = false;
            while (!stop) {
                stop = wobbleSwing.isBusy() == false;
            }
            this.swingPosition = SwingPosition.LiftUp;
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setPower(0);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleSwing.setPower(-0.01);
        }
    }

    @BotAction(displayName = "Lift Wobble Wall", defaultReturn = "")
    public void liftWobbleWall() {
        if (wobbleSwing != null) {
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setTargetPosition(SWING_LIFT_WALL);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleSwing.setPower(0.7);
            boolean stop = false;
            while (!stop) {
                stop = wobbleSwing.isBusy() == false;
            }
            this.swingPosition = SwingPosition.LiftUp;
            wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleSwing.setPower(0);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleSwing.setPower(-0.005);
        }
    }


    @BotAction(displayName = "Green Light", defaultReturn = "")
    public void signalOK() {
        getLights().OK();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1) {

        }
        getLights().none();
    }


    public void initDetector(String side, LinearOpMode caller) {
        try {
            rf = new RingDetector(this.hwMap, side, caller, this.namedCoordinates, this.getLights(), telemetry);
        }
        catch (Exception ex){
            telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
            telemetry.update();
        }
    }

    public void stopDetection() {
        if (rf != null) {
            rf.stopDetection();
        }
    }

    public void initDetectorThread(String side, LinearOpMode caller) {
        try {
            rf = new RingDetector(this.hwMap, side, caller, this.namedCoordinates, this.getLights(), telemetry);
            Thread detectThread = new Thread(rf);
            detectThread.start();
        } catch (Exception ex) {
            telemetry.addData("Error", String.format("Unable to initialize Detector thread. %s", ex.getMessage()));
            telemetry.update();
        }
    }

    // compound claw up for wall
    @BotAction(displayName = "Lift Wall and Grab", defaultReturn = "")
    public void liftWallGrab() {
        ElapsedTime runtime = new ElapsedTime();
        closeWobbleClaw();
        while (runtime.milliseconds() <= 300) {
        }
        liftWobbleWall();
    }

    ///get results of detection on the thread
    @BotAction(displayName = "Get Detection Result", defaultReturn = "B")
    public AutoDot getDetectionResult() {
        AutoDot target = null;
        if (rf != null) {
            rf.stopDetection();
            target = rf.getRecogZone();
        }

        telemetry.addData("Detected Zone: ", target.getDotName());
        telemetry.addData("Detected X: ", target.getX());
        telemetry.addData("Detected Y: ", target.getY());
        telemetry.update();
        return target;
    }


    @BotAction(displayName = "Detection Lights", defaultReturn = "")
    public void displayDetectionLights() {
        rf.displayLights();
    }

    @BotAction(displayName = "Lights Off", defaultReturn = "")
    public void lightsOff() {
        getLights().none();
    }

    ///use for non-threaded detection
    @BotAction(displayName = "Detect Stack", defaultReturn = "B")
    public AutoDot detectStack(String side) {
        AutoDot target = null;
        if (rf != null) {
            try {
                target = rf.detectRing(2, side, telemetry, owner);
            } finally {
                if (rf != null) {
                    rf.stopDetection();
                }
            }
        }
        return target;
    }

    public SwingPosition getSwingPosition() {
        return swingPosition;
    }
}
