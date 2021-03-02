package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private Servo wobbleSwing = null;
    private Servo wobbleClaw1 = null;
    private Servo wobbleClaw2 = null;
    private Servo ringCamera = null;
    private Servo shooterServo = null;
    private Servo ringGuard = null;
    private DcMotorEx shooter = null;

    private SwingPosition swingPosition = SwingPosition.Init;
    private static double SWING_BACK_POS = 1;
    private static double SWING_PLACE_POS = 0.25;
    private static double SWING_LIFT_AND_HOLD = 0.45;
    private static double SWING_LIFT_WALL = 0.7;
    private static double SHOOT_SERVO = 0.7;

    private static int TIMEOUT = 2500;
    private static int TIMEOUT_LONGER = 3000;
    private static int TIMEOUT_SHORTER = 1500;


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
            DcMotorEx intake = getIntakeMotor();
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with intake. Check the controller config", ex);
        }

        try {
            shooter = hwMap.get(DcMotorEx.class, "shooter");
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(0);
        } catch (Exception ex) {
            throw new Exception("Issues with shooter. Check the controller config", ex);
        }
        // claw starts closed
        try {
            wobbleClaw1 = hwMap.get(Servo.class, "claw1");
            wobbleClaw1.setPosition(0.3);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw1. Check the controller config", ex);
        }

        try {
            wobbleClaw2 = hwMap.get(Servo.class, "claw2");
            wobbleClaw2.setPosition(0.7);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw2. Check the controller config", ex);
        }
        // camera has no init position so can manually move
        try {
            ringCamera = hwMap.get(Servo.class, "camera");
        } catch (Exception ex) {
            throw new Exception("Issues with ringCamera. Check the controller config", ex);
        }

        try {
            shooterServo = hwMap.get(Servo.class, "shoot");
            shooterServo.setPosition(SHOOT_SERVO);
        } catch (Exception ex) {
            throw new Exception("Issues with shooterServo. Check the controller config", ex);
        }

        try {
            wobbleSwing = hwMap.get(Servo.class, "wobble");
            wobbleSwing.setPosition(SWING_BACK_POS);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleSwing. Check the controller config", ex);
        }

        try {
            ringGuard = hwMap.get(Servo.class, "rguard");
            ringGuard.setPosition(1);
        } catch (Exception ex) {
            throw new Exception("Issues with ringGuard. Check the controller config", ex);
        }

        telemetry.addData("Init", "Ultimate is ready");
    }

    public DcMotorEx getIntakeMotor(){
        return rightOdo;
    }

    public double getShooterVelocity(){
        return shooter.getVelocity();
    }


    @BotAction(displayName = "Move Intake", defaultReturn = "")
    public void intake() {
        DcMotorEx intake = getIntakeMotor();
        if (intake != null) {
            intake.setPower(0.9);
        }
    }

    @BotAction(displayName = "Move Intake Reverse", defaultReturn = "")
    public void intakeReverse() {
        DcMotorEx intake = getIntakeMotor();
        if (intake != null) {
            intake.setPower(-0.7);
        }
    }

    @BotAction(displayName = "Stop Intake", defaultReturn = "")
    public void stopintake() {
        DcMotorEx intake = getIntakeMotor();
        if (intake != null) {
            intake.setPower(0);
        }
    }

    @BotAction(displayName = "Move Shooter", defaultReturn = "")
    public void shooter() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.835);
        }
    }

    @BotAction(displayName = "Move Peg Shooter", defaultReturn = "")
    public void shooterpeg() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.77);
        }
    }

    @BotAction(displayName = "Move B Shooter", defaultReturn = "")
    public void shooterB() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.8);
        }
    }

    @BotAction(displayName = "Move Slower Shooter", defaultReturn = "")
    public void shooterslower() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.75);
        }
    }

    @BotAction(displayName = "Shoot", defaultReturn = "")
    public void shootServo() {
        ElapsedTime runtime = new ElapsedTime();
        if (shooterServo != null) {
            shooterServo.setPosition(SHOOT_SERVO - 0.4);
            runtime.reset();
            while (runtime.milliseconds() <= 250) {

            }
            shooterServo.setPosition(SHOOT_SERVO);
        }
    }

    @BotAction(displayName = "Guard Down", defaultReturn =  "")
    public void guardDown() {
        ringGuard.setPosition(0);
    }

    @BotAction(displayName = "Guard Up", defaultReturn =  "")
    public void guardUp() {
        ringGuard.setPosition(1);
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
            wobbleClaw1.setPosition(0.3);
            wobbleClaw2.setPosition(0.7);
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

    // WOBLLE FUNCTIONS
    @BotAction(displayName = "Wobble Little Up", defaultReturn = "")
    public void wobbleLittleUp() {
        if (wobbleSwing != null) {
            double currposition = wobbleSwing.getPosition();
            currposition = currposition + 0.05;
            wobbleSwing.setPosition(currposition);
        }
    }

    @BotAction(displayName = "Wobble Little Down", defaultReturn = "")
    public void wobbleLittleDown() {
        if (wobbleSwing != null) {
            double currposition = wobbleSwing.getPosition();
            currposition = currposition - 0.05;
            wobbleSwing.setPosition(currposition);
        }
    }

    @BotAction(displayName = "Init WobbleSwing", defaultReturn = "")
    public void backWobbleSwing() {
        if (wobbleSwing != null) {
            wobbleSwing.setPosition(SWING_BACK_POS);
        }
    }

    @BotAction(displayName = "Place Wobble", defaultReturn = "")
    public void forwardWobbleSwing() {
        if (wobbleSwing != null) {
            wobbleSwing.setPosition(SWING_PLACE_POS);
        }
    }

    @BotAction(displayName = "Lift Wobble Up Hold", defaultReturn = "")
    public void liftAndHoldWobbleSwing() {
        if (wobbleSwing != null) {
            wobbleSwing.setPosition(SWING_LIFT_AND_HOLD);
        }
    }

    @BotAction(displayName = "Lift Wobble Wall", defaultReturn = "")
    public void liftWobbleWall() {
        if (wobbleSwing != null) {
            wobbleSwing.setPosition(SWING_LIFT_WALL);
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

    public double getWobblePos() {
        if (wobbleSwing != null) {
            return wobbleSwing.getPosition();
        } else {
            return (-1);
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
//                if (rf != null) {
//                    rf.stopDetection();
//                }
            }
        }
        return target;
    }

}
