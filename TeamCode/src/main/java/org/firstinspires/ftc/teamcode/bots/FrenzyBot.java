package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;

public class FrenzyBot extends FrenzyBaseBot {
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx rotator = null;
    private Servo dropperServo = null;
    private static final String TAG = "FrenzyBot";
    private static int LIFT_FULL_EXTENSION = -1400;
    private static int LIFT_HALF_EXTENSION = -650;
    private static int LIFT_NO_EXTENSION = -10;
    private static double LIFT_SPEED = 0.8;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_READY = 0.5;
    private static double DROPPER_SERVO_POS_DROP = 0.0;

    // Detection
    CVDetector detector;

    private GameElement detectedElement;

    /* Constructor */
    public FrenzyBot() {

    }

    @Override
    public void init(LinearOpMode owner, HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        super.init(owner, ahwMap, telemetry);
        try {
            intake = hwMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize intake", ex);
        }
        try {
            lift = hwMap.get(DcMotorEx.class, "lift");
            lift.setDirection(DcMotor.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize lift", ex);
        }
        try {
            rotator = hwMap.get(DcMotorEx.class, "rotator");
            rotator.setDirection(DcMotor.Direction.FORWARD);
            rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotator.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize rotator", ex);
        }
        try {
            dropperServo =  hwMap.get(Servo.class, "dropper");
            dropperServo.setPosition(DROPPER_SERVO_POS_READY);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize dropperServo", ex);
        }
        try {
            detector = new CVDetector(hwMap);
            detector.init(CVDetectMode.Frenzy, "wcam", "cameraMonitorViewId");
            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize Detector", ex);
        }
    }

    public GameElement getDetection() {
        detectedElement = detector.getGameElement();
        telemetry.addData("Element: ", detectedElement);
        telemetry.addData("Mean Val: ", detector.getMeanVal());
        telemetry.update();
        detector.stopDetection();
        return detectedElement;
    }

    public void activateIntake(double velocity) {
        if (intake != null) {
            intake.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateLift(double velocity) {
        if (lift != null) {
            lift.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateRotator(double velocity) {
        if (rotator != null) {
            rotator.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }

    public int getLiftPosition(){
        return this.lift.getCurrentPosition();
    }

    public void liftToUpper(){
        this.lift.setTargetPosition(LIFT_FULL_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    public void liftToMid(){
        this.lift.setTargetPosition(LIFT_HALF_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    public void liftToLower(){
        this.lift.setTargetPosition(LIFT_NO_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    public void dropElement(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_DROP);
        }
    }

    public void resetDropper(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_READY);
        }
    }

    public void startIntake() {
        activateIntake(0.95);
    }

    public void reverseIntake() {
        activateIntake(-0.75);
    }

    public void stopIntake() {
        activateIntake(0);
    }

    public void startTurntableBlue() {
        activateRotator(0.75);
    }
    public void startTurntableRed() {
        activateRotator(-0.75);
    }
    public void stopTurntable() {
        activateRotator(0.0);
    }
}
