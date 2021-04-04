package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;
import android.util.Log;

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
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import org.firstinspires.ftc.teamcode.skills.TurretAngler;

import static java.lang.Math.abs;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

public class UltimateBot extends YellowBot {
    private Servo wobbleSwing = null;
    private Servo wobbleClaw1 = null;
    private Servo wobbleClaw2 = null;
    private Servo shooterServo = null;
    private Servo ringGuard = null;
    private Servo turretServo = null;
    private Servo camera = null;
    private DcMotorEx shooter = null;

    private static double SWING_BACK_POS = 1;
    private static double SWING_PLACE_POS = 0.2;
    private static double SWING_LIFT_AND_HOLD = 0.35;
    private static double SWING_LIFT_WALL = 0.45;
    private static double SHOOT_SERVO = 0.6;

    private boolean syncturretcamera = true;

    private RingDetector rf = null;
    private TurretAngler ta = null;


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
            DcMotorEx intakecurve = getIntakeCMotor();
            intakecurve.setDirection(DcMotor.Direction.REVERSE);
            intakecurve.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakecurve.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with shooter. Check the controller config", ex);
        }

        try {
            shooter = hwMap.get(DcMotorEx.class, "shooter");
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(0);
        } catch (Exception ex) {
            throw new Exception("Issues with shooter. Check the controller config", ex);
        }
        // claw starts closed
        try {
            wobbleClaw1 = hwMap.get(Servo.class, "claw1");
            wobbleClaw1.setPosition(0);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw1. Check the controller config", ex);
        }

        try {
            wobbleClaw2 = hwMap.get(Servo.class, "claw2");
            wobbleClaw2.setPosition(1);
        } catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw2. Check the controller config", ex);
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
            ringGuard.setPosition(0.7);
        } catch (Exception ex) {
            throw new Exception("Issues with ringGuard. Check the controller config", ex);
        }

        try {
            turretServo = hwMap.get(Servo.class, "turret");
            turretServo.setPosition(0.5);
        } catch (Exception ex) {
            throw new Exception("Issues with turret. Check the controller config", ex);
        }

        try {
            camera = hwMap.get(Servo.class, "camera");
            camera.setPosition(0.5);
        } catch (Exception ex) {
            throw new Exception("Issues with camera. Check the controller config", ex);
        }

        telemetry.addData("Init", "Ultimate is ready");
    }

    public DcMotorEx getIntakeMotor(){
        return rightOdo;
    }

    public DcMotorEx getIntakeCMotor() {
        return leftOdo;
    }

    public double getShooterVelocity(){
        return shooter.getVelocity();
    }


    // INTAKE FUNCTIONS
    @BotAction(displayName = "Move Intake", defaultReturn = "")
    public void intake() {
        DcMotorEx intake = getIntakeMotor();
        DcMotorEx intakecurve = getIntakeCMotor();
        if (intake != null) {
            intake.setPower(0.9);
            intakecurve.setPower(0.9);
        }
    }

    @BotAction(displayName = "Move Intake Reverse", defaultReturn = "")
    public void intakeReverse() {
        DcMotorEx intake = getIntakeMotor();
        DcMotorEx intakecurve = getIntakeCMotor();
        if (intake != null) {
            intake.setPower(-0.7);
            intakecurve.setPower(-0.7);
        }
    }

    @BotAction(displayName = "Stop Intake", defaultReturn = "")
    public void stopintake() {
        DcMotorEx intake = getIntakeMotor();
        DcMotorEx intakecurve = getIntakeCMotor();
        if (intake != null) {
            intake.setPower(0);
            intakecurve.setPower(0);
        }
    }


    // MOVE SHOOTER FUNCTIONS
    @BotAction(displayName = "Move Shooter", defaultReturn = "")
    public void shooter() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.85);
        }
    }

    @BotAction(displayName = "Move Shooter High", defaultReturn = "")
    public void shooterhigh() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.8);
        }
    }

    @BotAction(displayName = "Move Shooter Med", defaultReturn = "")
    public void shootermed() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.7);
        }
    }

    @BotAction(displayName = "Move Shooter Low", defaultReturn = "")
    public void shooterlow() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.75);
        }
    }

    @BotAction(displayName = "Move Shooter Lower", defaultReturn = "")
    public void shooterlower() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.73);
        }
    }

    @BotAction(displayName = "Move Peg Shooter Low", defaultReturn = "")
    public void shooterpeglow() {
        if (shooter != null) {
            shooter.setVelocity(MAX_VELOCITY*0.65);
        }
    }

    @BotAction(displayName = "Stop Shooter", defaultReturn = "")
    public void stopshooter() {
        if (shooter != null) {
            shooter.setPower(0);
        }
    }

    // SHOOT SERVO
    @BotAction(displayName = "Shoot", defaultReturn = "")
    public void shootServo() {
        ElapsedTime runtime = new ElapsedTime();
        if (shooterServo != null) {
            shooterServo.setPosition(SHOOT_SERVO + 0.4);
            runtime.reset();
            while (runtime.milliseconds() <= 300) {

            }
            shooterServo.setPosition(SHOOT_SERVO);
        }
    }

    // GUARD WALL FUNCTIONS
    @BotAction(displayName = "Guard Down", defaultReturn =  "")
    public void guardDown() {
        ringGuard.setPosition(0.03);
    }

    @BotAction(displayName = "Guard Up", defaultReturn =  "")
    public void guardUp() {
        ringGuard.setPosition(0.7);
    }

    @BotAction(displayName = "Guard Middle", defaultReturn =  "")
    public void guardMiddle() {
        ringGuard.setPosition(0.5);
    }

    // CLAW FUNCTIONS
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

    // CAMERA POSITIONING
    double cameraPos = 0;
    @BotAction(displayName = "Camera Init", defaultReturn = "")
    public void cameraInit() {
        if (camera != null) {
            camera.setPosition(0.5);
        }
    }

    public void cameraRight() {
        if (camera != null) {
            cameraPos = camera.getPosition();
            cameraPos = cameraPos + 0.005;
            camera.setPosition(cameraPos);
        }
    }

    public void cameraLeft() {
        if (camera != null) {
            cameraPos = camera.getPosition();
            cameraPos = cameraPos - 0.005;
            camera.setPosition(cameraPos);
        }
    }

    // TURRET FUNCTIONS
    double turretPos = 0;
    @BotAction(displayName = "Turret Little Right", defaultReturn = "")
    public void turretLittleRight() {
        if (turretServo != null) {
            turretPos = turretServo.getPosition();
            turretPos = turretPos + 0.005;
            turretServo.setPosition(turretPos);
        }
    }

    @BotAction(displayName = "Turret Little Left", defaultReturn = "")
    public void turretLittleLeft() {
        if (turretServo != null) {
            turretPos = turretServo.getPosition();
            turretPos = turretPos - 0.005;
            turretServo.setPosition(turretPos);
        }
    }

    @BotAction(displayName = "Turret Init", defaultReturn = "")
    public void turretInit() {
        if (turretServo != null) {
            turretServo.setPosition(0.5);
        }
    }

    @BotAction(displayName = "Turret to Camera", defaultReturn = "")
    public void turretCamera() {
        if (turretServo != null) {
            if (cameraPos > 0.5) {
                turretPos = cameraPos * 0.95;
            } else if (cameraPos < 0.5) {
                turretPos = cameraPos * 1.05;
            } else {
                turretPos = 0.5;
            }
            turretServo.setPosition(turretPos);
        }
    }

    // TURRET ANGLING THREAD FUNCTIONS
    public void initTurretThread(LinearOpMode caller) {
        try {
            ta = new TurretAngler(this.hwMap, caller, telemetry);
            Thread turretThread = new Thread(ta);
            turretThread.start();
        } catch (Exception ex) {
            telemetry.addData("Error", String.format("Unable to initialize Turret thread. %s", ex.getMessage()));
            telemetry.update();
        }
    }

    public void activatTurretThread(LinearOpMode caller) {
        try {
            if (ta !=null) {
                ta.activateTracker();
                Thread turretThread = new Thread(ta);
                turretThread.start();
            }
        } catch (Exception ex) {
            telemetry.addData("Error", String.format("Unable to activate Turret thread. %s", ex.getMessage()));
            telemetry.update();
        }
    }

    int cameramove = 0;
    boolean turretrunning = true;

    @BotAction(displayName = "Change Turret Sync", defaultReturn = "")
    public void changeTurretSync() {
        syncturretcamera = !syncturretcamera;
    }

    public void angleTurret() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if (ta != null) {
            while (turretrunning) {
                cameramove = ta.moveTurret();
                if (cameramove == 1) {
                    cameraLeft();
                    if (syncturretcamera) {
                        turretCamera();
                    }
                } else if (cameramove == 2) {
                    cameraRight();
                    if (syncturretcamera) {
                        turretCamera();
                    }
                }
                while(timer.milliseconds() < 60){
                }
                timer.reset();
            }
        }
    }

    public void stopTurretAngler() {
        if (ta != null) {
            ta.stopThread();
            turretrunning = false;
        }
    }


    // RING RECOGNITION FUNCTIONS
    public void initDetector(String side, LinearOpMode caller) {
        try {
            rf = new RingDetector(this.hwMap, side, caller, this.namedCoordinates, telemetry);
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
            rf = new RingDetector(this.hwMap, side, caller, this.namedCoordinates, telemetry);
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


    ///use for non-threaded detection
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


    // SHOOT PEG FUNCTION
    public void shootPegTurn(RobotCoordinatePosition locator){
        // start shooter
        shooterpeglow();
        turretInit();

        //wait for the locator to stabilize
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 500 && this.owner.opModeIsActive()){
        }

        double originalOrientation = locator.getAdjustedCurrentHeading();
        double strafeSpeed = 0.2;
        double spinSpeed = 0.1;
        double strafeTo = 15;

        strafeTo(strafeSpeed, strafeTo, true);

        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        double newOrientation = locator.getAdjustedCurrentHeading();
        int marginError = 3;
        Log.d("UltimateBot", String.format("newOrientation 1: %.2f", newOrientation));
        //spin to the desired orientation
        double diff = newOrientation - originalOrientation;
        if (abs(diff) > marginError) {
            double updated = originalOrientation + diff/2;
            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(updated, spinSpeed, locator);
            spin(profileSpin, locator);
        }

        shootServo();
        turretServo.setPosition(0.4);
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        shootServo();
        turretServo.setPosition(0.6);
        timer.reset();
        while(timer.milliseconds() < 500 && this.owner.opModeIsActive()){
        }
        shootServo();

    }

    public void shootPegSequence(RobotCoordinatePosition locator){
        //start shooter
        shooterpeglow();
        turretInit();

        //wait for the locator to stabilize
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }

        //get the orientation as the locator knows it. we'll use it later for corrections
        double originalOrientation = locator.getAdjustedCurrentHeading();
        Log.d("UltimateBot", String.format("original orientation: %.2f", originalOrientation));
        double strafeSpeed = 0.4;
        double strafeSpeedFirst = 0.35;
        double spinSpeed = 0.1;
        double strafeToFirst = 8;
        double strafeBetweenPegs = 2.2;
        double strafeToLastPeg = 2;

        //FirstPeg
        //strafe to align the robot with the first peg
        strafeTo(strafeSpeedFirst, strafeToFirst, true);
        //let locator settle
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        double newOrientation = locator.getAdjustedCurrentHeading();
        int marginError = 3;
        Log.d("UltimateBot", String.format("newOrientation 1: %.2f", newOrientation));
        //spin to the desired orientation
        double diff = newOrientation - originalOrientation;
        if (abs(diff) > marginError) {
            double updated = originalOrientation + diff/2;
            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(updated, spinSpeed, locator);
            spin(profileSpin, locator);
        }
        //shoot
        shootServo();

        //Second Peg
        //strafe to the next peg
        strafeTo(strafeSpeed, strafeBetweenPegs, true);
//        //let locator settle
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        newOrientation = locator.getAdjustedCurrentHeading();
        Log.d("UltimateBot", String.format("newOrientation 2: %.2f", newOrientation));
//        spin to the desired orientation
//        diff = newOrientation - originalOrientation;
//        if (abs(diff ) > marginError) {
//            double updated = originalOrientation + diff/2;
//            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(updated, spinSpeed, locator);
//            spin(profileSpin, locator);
//        }
        //shoot
        shootServo();

        //Third Peg
        //strafe to the next peg
        strafeTo(strafeSpeed, strafeToLastPeg, true);
        //let locator settle
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        marginError = 1;
        newOrientation = locator.getAdjustedCurrentHeading();
        Log.d("UltimateBot", String.format("newOrientation 2: %.2f", newOrientation));
        //spin to the desired orientation
//        diff = newOrientation - originalOrientation;
//        if (Math.abs(diff) > marginError) {
//            double updated = newOrientation - 2;
//            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(updated, spinSpeed, locator);
//            spin(profileSpin, locator);
//        }
        //shoot
        shootServo();

    }

    public void shootPegSequenceManual(RobotCoordinatePosition locator){
        //start shooter
        shooterpeglow();

        //wait for the locator to stabilize
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 750 && this.owner.opModeIsActive()){
        }

        //get the orientation as the locator knows it. we'll use it later for corrections
        double strafeSpeed = 0.25;
        double strafeBetweenPegs = 2.5;

        // shoot first peg
        shootServo();

        // strafe to second peg
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }

        strafeTo(strafeSpeed, strafeBetweenPegs, true);

        //shoot
        shootServo();


        // strafe to the next peg
        timer.reset();
        while(timer.milliseconds() < 300 && this.owner.opModeIsActive()){
        }
        strafeTo(strafeSpeed, strafeBetweenPegs, true);

        //shoot
        shootServo();

    }
}
