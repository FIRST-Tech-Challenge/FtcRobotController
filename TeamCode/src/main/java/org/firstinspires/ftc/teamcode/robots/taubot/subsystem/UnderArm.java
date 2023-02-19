package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;



import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.ELBOW_TO_WRIST;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.SHOULDER_TO_ELBOW;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.*;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.taubot.simulation.ServoSim;
import org.firstinspires.ftc.teamcode.util.Vector3;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPUnderArm")
public class UnderArm implements Subsystem {
    private Robot robot;
    public static int SHOULDER_START_ANGLE = 110;
    public static int SHOULDER_HOME_PWM = 1540;
    public static int ELBOW_HOME_PWM = 1520;
    public static int WRIST_HOME_PWM = 1500;
    public static int TURRET_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = (2013-SHOULDER_HOME_PWM)/90.0; //eyeball calibration 1/16/23
    public static double ELBOW_PWM_PER_DEGREE = (2030-ELBOW_HOME_PWM)/90.0;
    public static double WRIST_PWM_PER_DEGREE = 750.0 / 180.0; //todo if we need it
    public static double TURRET_PWM_PER_DEGREE = 750.0 / 180.0; //todo

    public static double kF = 0.0;

    public static double SHOULDER_DEG_MIN = -60; // negative angles are counter clockwise while looking at the left side
    public static double SHOULDER_DEG_MAX = 90; // of the robot

    public static double ELBOW_DEG_MIN = -80;
    public static double WRIST_DEG_MIN = -180;
    public static double TURRET_DEG_MIN = -360;

    public static double ELBOW_DEG_MAX = 140;
    public static double WRIST_DEG_MAX = 180;
    public static double TURRET_DEG_MAX = 360;

    public static double LASSO_CLOSED = 1150;
    public static double LASSO_OPEN = 2500;

    public static double TRANSFER_SHOULDER_ANGLE = 0;
    public static double TRANSFER_ELBOW_ANGLE = 0;

    boolean lassoGripped = false;

    public static double UNDERARM_HEIGHT = 7; //height of rotation of shoulder motor   INCHES

    //these 5 config variables won't have any effect if changed in dashboard since they are currently assigned at compile

    public static double P = 0.995;

    public Servo elbowServo, wristServo, lassoServo;
    public Servo shoulderServo, turretServo;

    private double chariotDistance;
    private double shoulderAngle;
    private double shoulderTargetAngle, elbowTargetAngle, wristTargetAngle, turretTargetAngle;

    private Articulation articulation;

    public UnderArm(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);

        if (simulated) { //if (simulated) { ignoring simulation as a way to test real underarm off robot
            shoulderServo = new ServoSim();
            elbowServo = new ServoSim();
            wristServo = new ServoSim();
            lassoServo = new ServoSim();
            turretServo = new ServoSim();
        } else {
            shoulderServo = hardwareMap.get(ServoImplEx.class, "shoulderServo");
            ((ServoImplEx) shoulderServo).setPwmRange(axonRange);
            elbowServo = hardwareMap.get(ServoImplEx.class, "elbowServo");
            ((ServoImplEx) elbowServo).setPwmRange(axonRange);
            wristServo = hardwareMap.get(ServoImplEx.class, "wristServo");
            lassoServo = hardwareMap.get(ServoImplEx.class, "lassoServo");
            ((ServoImplEx) lassoServo).setPwmRange(axonRange);
            turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
            ((ServoImplEx) turretServo).setPwmRange(axonRange);
        }

        shoulderTargetAngle = 0;
        elbowTargetAngle = 0;
        wristTargetAngle = 0;
        turretTargetAngle = 0;

        articulation = Articulation.noIK;
        fieldPositionTarget = getUnderArmPosition(); //crane default IK starting point is
    }

    Vector3 underArmPosition = new Vector3(0,0,0);

    public Vector3 getUnderArmPosition(){
        return underArmPosition;        //returns the calculated underarm position
    }

    Vector3 fieldPositionTarget = new Vector3(0,0,0);

    public enum Articulation {
        transfer,
        home,
        manual,
        noIK,
        fold
    }

    public static double FOLDPOS_SHOULDER_ANGLE = 0;
    public static double FOLDPOS_ELBOW_ANGLE = 0;
    public static double FOLDPOS_TURRET_ANGLE = 0;

    public Articulation articulate(Articulation target){
        articulation = target;

        switch(articulation){
            case fold: //PROB NOT NEEDED but added just in case
                setShoulderTargetAngle(FOLDPOS_SHOULDER_ANGLE);
                setElbowTargetAngle(FOLDPOS_ELBOW_ANGLE);
                setTurretTargetAngle(FOLDPOS_TURRET_ANGLE);
                break;
            case transfer: //transfer position
                goToTransfer();
                break;
            case home: //safe position for transiting the field
                goHome();
                break;
            case manual: //normal IK intake mode
                holdTarget(fieldPositionTarget.x,fieldPositionTarget.y,fieldPositionTarget.z);
                break;
            case noIK: //no targets are set

                break;
        }

        return target;
    }

    public void holdTarget(double x, double y, double z){
        calculateFieldTargeting(x,y,z);
        goToCalculatedTarget();
    }

    double targetHeight;
    double targetDistance;

    public void goToCalculatedTarget(){
        setTurretTargetAngle(calculatedTurretAngle);
        targetHeight = calculatedHeight;
        targetDistance = calculatedDistance;
        setShoulderTargetAngle(calculatedShoulderAngle);
        setElbowTargetAngle(calculatedElbowAngle);
    }

    long homeTimer;
    int homeStage = 0;

    public boolean goHome(){

        switch (homeStage) {
            case 0: //sets home position
                robot.driveTrain.tuck();
                setTurretTargetAngle(0);
                setElbowTargetAngle(0);
                setShoulderTargetAngle(0);
                homeTimer = futureTime(0.5);
                homeStage++;
                break;
            case 1:
                if (System.nanoTime() > homeTimer) {
                    homeStage = 0;
                    return true;
                }
                break;
        }

        return false;
    }

    long transferTimer;
    int transferStage = 0;
    boolean atTransfer = false;
    public boolean goToTransfer(){
        switch (transferStage) {
            case 0: //sets home position
                atTransfer = false;
                robot.driveTrain.tuck();
                setTurretTargetAngle(0);
                setElbowTargetAngle(TRANSFER_ELBOW_ANGLE);
                setShoulderTargetAngle(TRANSFER_SHOULDER_ANGLE);
                transferTimer = futureTime(0.5);
                transferStage++;
                break;
            case 1:
                if (System.nanoTime() > transferTimer) {
                    transferStage = 0;
                    atTransfer = true;
                    return true;
                }
                break;
        }

        return false;
    }

    public boolean atTransfer(){
        return atTransfer;
    }

    double calculatedTurretAngle;
    double calculatedHeight;
    double calculatedDistance;
    double calculatedShoulderAngle;
    double calculatedElbowAngle;

    public static double shoulderHeight = 0.25;

    public static double UPPER_ARM_LENGTH = 14.06;
    public static double LOWER_ARM_LENGTH = 11.02;

    public boolean calculateFieldTargeting(double x, double y, double z){ //THIS IS IN INCHES!!!!!!!!

        z /= INCHES_PER_METER;

        double hypotToTarget;
        double locX;
        double locY;
//        double calculatedDistanceFieldUnits;

        locX = x - underArmPosition.x;
        locY = y - underArmPosition.y;


        calculatedTurretAngle = Math.toDegrees(Math.atan2(locY, locX));

        calculatedHeight = z-shoulderHeight;

        calculatedDistance = (Math.sqrt(Math.pow(locY,2) + Math.pow(locX,2)));

//        calculatedDistanceFieldUnits = (Math.sqrt(Math.pow(locY,2) + Math.pow(locX,2)));

        hypotToTarget = Math.sqrt(Math.pow(calculatedHeight, 2) + Math.pow(calculatedDistance, 2));

//        double virtualLength = Math.sqrt( Math.pow(calculatedDistance,2) + Math.pow(calculatedHeight,2) );

//        calculatedElbowAngle = Math.toDegrees(2*Math.asin( virtualLength / (SHOULDER_TO_ELBOW + ELBOW_TO_WRIST) ));
//        calculatedShoulderAngle = Math.toDegrees(Math.atan2( calculatedDistance , calculatedHeight ) + (90 - calculatedElbowAngle/2));

        //todo: hope and pray that this works
        calculatedShoulderAngle = Math.toDegrees(Math.acos(
                (Math.pow(UPPER_ARM_LENGTH, 2) + Math.pow(calculatedDistance, 2) + Math.pow(calculatedHeight, 2) - Math.pow(LOWER_ARM_LENGTH, 2))
                                        / (2 * UPPER_ARM_LENGTH * hypotToTarget))
                                        + Math.acos(calculatedDistance / hypotToTarget));

        calculatedElbowAngle = Math.toDegrees(Math.acos(
                                (Math.pow(UPPER_ARM_LENGTH, 2) + Math.pow(LOWER_ARM_LENGTH, 2) - Math.pow(calculatedDistance, 2) - Math.pow(calculatedHeight , 2))
                                                            / (2 * UPPER_ARM_LENGTH * LOWER_ARM_LENGTH)));

        return true;
    }

    //todo these adjust methods are horrid - they need to have range limits applied to them and time based velocity if we are keeping them

    public void adjustShoulder(double speed){
        shoulderTargetAngle -= 100*speed*robot.deltaTime;
    }

    public void adjustElbow(double speed){
        elbowTargetAngle += 100*speed*robot.deltaTime;
    }

    public void adjustLasso(double speed){
        wristTargetAngle += 100*speed*robot.deltaTime;
    }

    public void adjustTurret(double speed){
        turretTargetAngle += 400*speed*robot.deltaTime;
    }

    public static double ADJUST_HEIGHT_SPEED = 2;
    public static double ADJUST_POSITION_SPEED = 2;

    public void adjustX(double speed){
        fieldPositionTarget.x += ADJUST_POSITION_SPEED*speed;
    }

    public void adjustY(double speed){
        fieldPositionTarget.y += ADJUST_POSITION_SPEED*speed;
    }

    public void adjustZ(double speed){
        fieldPositionTarget.z += ADJUST_HEIGHT_SPEED*speed;
    }
    @Override
    public void update(Canvas fieldOverlay) {

        Pose2d robotPosInches = robot.driveTrain.getPoseEstimate();
        double headingRad = robot.driveTrain.getRawHeading();
        double underArmLengthInches = robot.driveTrain.getChassisLength();

        underArmPosition =  new Vector3(robotPosInches.getX()+underArmLengthInches*Math.cos(headingRad),
                robotPosInches.getY()+underArmLengthInches*Math.sin(headingRad),
                UNDERARM_HEIGHT); //calculates and sets the position of underarm in world coordinates

        chariotDistance = robot.driveTrain.getChassisLength();

        articulate(articulation);



        shoulderServo.setPosition(servoNormalizeExtended(shoulderServoValue(shoulderTargetAngle)));
        elbowServo.setPosition(servoNormalizeExtended(elbowServoValue(elbowTargetAngle)));
        wristServo.setPosition(servoNormalizeExtended(wristServoValue(wristTargetAngle)));
        turretServo.setPosition(servoNormalizeExtended(turretServoValue(turretTargetAngle)));
    }

    public void grip(){
        lassoGripped = true;
        lassoServo.setPosition(servoNormalizeExtended(LASSO_CLOSED));
    }

    public void release(){
        lassoGripped = false;
        lassoServo.setPosition(servoNormalizeExtended(LASSO_OPEN));
    }

    public void toggleLasso(){
        if(lassoGripped){
            release();
        }else{
            grip();
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public String getTelemetryName() {
        return "UnderArm";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);

        if (debug) {
            telemetryMap.put("Shoulder Target Angle", shoulderTargetAngle);
            telemetryMap.put("Elbow Target Angle", elbowTargetAngle);
            telemetryMap.put("Wrist Target Angle", wristTargetAngle);
            telemetryMap.put("Turret Target Angle", turretTargetAngle);
            telemetryMap.put("Under Arm Pos X", underArmPosition.x);
            telemetryMap.put("Under Arm Pos Y", underArmPosition.y);
            telemetryMap.put("Under Arm Target X", fieldPositionTarget.x);
            telemetryMap.put("Under Arm Target Y", fieldPositionTarget.y);
            telemetryMap.put("Under Arm Target Z", fieldPositionTarget.z);

            telemetryMap.put("Elbow Target PWM", elbowServoValue(elbowTargetAngle));
            telemetryMap.put("Shoulder Target PWM", shoulderServoValue(shoulderTargetAngle));
            telemetryMap.put("Wrist Target PWM", wristServoValue(wristTargetAngle));
            telemetryMap.put("Turret Target PWM", turretServoValue(turretTargetAngle));
            telemetryMap.put("Lasso Target PWM", lassoServo.getPosition());
            telemetryMap.put("Lasso Grip", lassoGripped);

            telemetryMap.put("chariot distance", getChariotDistance());

            double shoulderAngleRads = wrapAngleRad(Math.toRadians(90 - shoulderTargetAngle));
            double elbowAngleRads = -wrapAngleRad(Math.toRadians(180 - elbowTargetAngle));
            double wristAngleRads = wrapAngleRad(Math.toRadians(180) - wrapAngleRad(-elbowAngleRads + Math.toRadians(wristTargetAngle)));

            telemetryMap.put("horizontal distance", SHOULDER_TO_ELBOW * Math.cos(shoulderAngleRads) + ELBOW_TO_WRIST * Math.cos(shoulderAngleRads + elbowAngleRads));
            telemetryMap.put("vertical distance", SHOULDER_TO_ELBOW * Math.sin(shoulderAngleRads) + ELBOW_TO_WRIST * Math.sin(shoulderAngleRads + elbowAngleRads));
            telemetryMap.put("wrist absolute angle", wristAngleRads);
        }

        return telemetryMap;
    }


    private void setTargetPositions(double shoulderPos, double elbowPos, double wristPos) {
        setShoulderTargetAngle(shoulderPos);
        setElbowTargetAngle(elbowPos);
        setWristTargetAngle(wristPos);
    }
    // ----------------------------------------------------------------------------------------------
    // Getters And Setters
    // ----------------------------------------------------------------------------------------------

    private double shoulderServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newPos = newPos * SHOULDER_PWM_PER_DEGREE + SHOULDER_HOME_PWM;
        return newPos;
    }

    private double elbowServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, ELBOW_DEG_MIN, ELBOW_DEG_MAX);
        newPos = newPos * ELBOW_PWM_PER_DEGREE + ELBOW_HOME_PWM;
        return newPos;
    }

    private double wristServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, WRIST_DEG_MIN, WRIST_DEG_MAX);
        newPos = newPos * WRIST_PWM_PER_DEGREE + WRIST_HOME_PWM;
        return newPos;
    }

    private double turretServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, TURRET_DEG_MIN, TURRET_DEG_MAX);
        newPos = newPos * TURRET_PWM_PER_DEGREE + TURRET_HOME_PWM;
        return newPos;
    }

    public void setShoulderTargetAngle(double shoulderTargetAngle) {
        this.shoulderTargetAngle = wrapAngle(shoulderTargetAngle);
    }

    public void setElbowTargetAngle(double elbowTargetAngle) {
        this.elbowTargetAngle = wrapAngle(elbowTargetAngle);
    }

    public void setWristTargetAngle(double wristTargetAngle) {
        this.wristTargetAngle = wrapAngle(wristTargetAngle);
    }

    public void setTurretTargetAngle(double turretTargetAngle) {
        this.turretTargetAngle = wrapAngle(turretTargetAngle);
    }

    public double getShoulderTargetAngle() {
        return shoulderTargetAngle;
    }

    public double getElbowTargetAngle() {
        return elbowTargetAngle;
    }

    public double getWristTargetAngle() {
        return wristTargetAngle;
    }

    public Articulation getArticulation() {
        return articulation;
    }

    public double getChariotDistance() {
        return chariotDistance;
    }
}
