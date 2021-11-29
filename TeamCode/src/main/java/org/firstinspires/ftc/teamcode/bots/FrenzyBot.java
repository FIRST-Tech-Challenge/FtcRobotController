package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;

public class FrenzyBot extends FrenzyBaseBot {
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx rotatorRight = null;
    private DcMotorEx rotatorLeft = null;
    private Servo dropperServo = null;
    private static final String TAG = "FrenzyBot";
    public static int LIFT_LEVEL_THREE = -1930;
    public static int LIFT_LEVEL_TWO = -1190;
    public static int LIFT_LEVEL_ONE = -650;
    public static int LIFT_NO_EXTENSION = 0;

    private int liftLocation = LIFT_NO_EXTENSION;
    private static double LIFT_SPEED = 0.95;
    private static double LIFT_SPEED_LOW = 0.7;

    NormalizedColorSensor colorSensor;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_READY = 0.75;
    private static double DROPPER_SERVO_POS_MOVE = 0.6;
    private static double DROPPER_SERVO_POS_DROP = 0.0;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;
    private GameElement detectedElement;


    /* Constructor */
    public FrenzyBot() {
        opModeSide = AutoRoute.NAME_RED; // default
    }

    public FrenzyBot(String fieldSide) {
        this.opModeSide = fieldSide;
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
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize lift", ex);
        }
        try {
            rotatorRight = hwMap.get(DcMotorEx.class, "rotatorRight");
            rotatorRight.setDirection(DcMotor.Direction.FORWARD);
            rotatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatorRight.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize rotator", ex);
        }
        try {
            rotatorLeft = hwMap.get(DcMotorEx.class, "rotatorLeft");
            rotatorLeft.setDirection(DcMotor.Direction.FORWARD);
            rotatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatorLeft.setVelocity(0);
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
            colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize colorSensor", ex);
        }

    }

    @Override
    public void initDetectorThread(String side, LinearOpMode caller) {
        try {
            detector = new CVDetector(hwMap, opModeSide, CVDetectMode.Frenzy, this.namedCoordinates);
            detector.startDetection();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize Detector", ex);
        }
    }

    @BotAction(displayName = "Get Detection Result", defaultReturn = "C")
    @Override
    public AutoDot getDetectionResult() {
        AutoDot level = detector.getLevel();
        Log.d(TAG, String.format("Detection result: Level %s", level.getDotName()));
        telemetry.addData("Level: ", level);
        telemetry.update();
        detector.stopDetection();

        return level;
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
    public void activateRotatorRight(double velocity) {
        if (rotatorRight != null) {
            rotatorRight.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateRotatorLeft(double velocity) {
        if (rotatorLeft != null) {
            rotatorLeft.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }

    public int getLiftPosition(){

        return this.lift.getCurrentPosition();
    }

    @BotAction(displayName = "Lift level 3", defaultReturn = "")
    public void liftToLevel3(){
        liftLocation = LIFT_LEVEL_THREE;
        prepDropperToMove();
        this.lift.setTargetPosition(LIFT_LEVEL_THREE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift level 2", defaultReturn = "")
    public void liftToLevel2(){
        liftLocation = LIFT_LEVEL_TWO;
        this.lift.setTargetPosition(LIFT_LEVEL_TWO);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift level 1", defaultReturn = "")
    public void liftToLevel1(){
        liftLocation = LIFT_LEVEL_ONE;
        prepDropperToMove();
        this.lift.setTargetPosition(LIFT_LEVEL_ONE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift to lower", defaultReturn = "")
    public void liftToLower(){
        //reset dropper before retracting the lift all the way
        resetDropper();
        if (liftLocation == LIFT_LEVEL_ONE){
            ElapsedTime time = new ElapsedTime();
            time.reset();
            //give some time for the dropper to close
            while (time.milliseconds() < 500){

            }
        }
        liftLocation = LIFT_NO_EXTENSION;
        this.lift.setTargetPosition(LIFT_NO_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (getLiftPosition() == LIFT_LEVEL_ONE){
            this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED_LOW);
        }
        else {
            this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED);
        }
    }

    public boolean isLiftBusy(){
        return this.lift.isBusy();
    }

    private void stopLift(){
        this.lift.setPower(0);
    }

    public int getLiftLocation(){
        return liftLocation;
    }

    @BotAction(displayName = "Drop element", defaultReturn = "")
    public void dropElement(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_DROP);
        }
    }

    @BotAction(displayName = "Reset dropper", defaultReturn = "")
    public void resetDropper(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_READY);
        }
    }

    private void prepDropperToMove(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_MOVE);
        }
    }

    @BotAction(displayName = "Start intake", defaultReturn = "")
    public void startIntake() {
        activateIntake(0.95);
    }

    @BotAction(displayName = "Reverse intake", defaultReturn = "")
    public void reverseIntake() {
        activateIntake(-0.75);
    }

    @BotAction(displayName = "Stop intake", defaultReturn = "")
    public void stopIntake() {
        activateIntake(0);
    }

    @BotAction(displayName = "Start turntable blue", defaultReturn = "")
    public void startTurntableBlue() {
        activateRotatorRight(0.75);
        activateRotatorLeft(0.75);
    }
    @BotAction(displayName = "Start turntable red", defaultReturn = "")
    public void startTurntableRed() {
        activateRotatorRight(-0.75);
        activateRotatorLeft(-0.75);
    }
    @BotAction(displayName = "Stop turntable", defaultReturn = "")
    public void stopTurntable() {
        activateRotatorRight(0.0);
        activateRotatorLeft(0.0);
    }
    private void toggleLight(boolean on){
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }
    public DetectedColor detectColor(Telemetry telemetry, float timeout) {
        toggleLight(true);
        // values is a reference to the hsvValues array.
        DetectedColor dc = DetectedColor.NONE;
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        runtime.reset();
        NormalizedRGBA colors = null;
        boolean stop = false;
        while(!stop) {
            // Read the sensor
            colors = colorSensor.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();
            if(colors.red > 0.2 && colors.green > 0.2 && colors.blue < 0.2){
                dc = DetectedColor.Yellow;
            }
            else if (colors.red > 0.2 && colors.green > 0.2 && colors.blue > 0.2){
                dc = DetectedColor.White;
            }
            stop = timeout == 0 || (timeout > 0 && runtime.seconds() >= timeout);
        }

        toggleLight(false);
        return dc;
    }
}
