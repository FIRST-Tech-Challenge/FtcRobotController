package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;



import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.ELBOW_TO_WRIST;
//import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MAX_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MIN_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MIN_SAFE_CHASSIS_LENGTH;
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
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Joint;
import org.firstinspires.ftc.teamcode.util.Vector3;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPUnderArm")
public class UnderArm implements Subsystem {
    private Robot robot;

    public static double _TuneableTimer = 1;

    public static int SHOULDER_START_ANGLE = 110;
    public static int SHOULDER_HOME_PWM = 1540;
    public static int ELBOW_HOME_PWM = 1520;
    public static int WRIST_HOME_PWM = 1500;
    public static int TURRET_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = 4.5; //eyeball calibration 1/16/23
    public static double ELBOW_PWM_PER_DEGREE = 4.5;
    public static double WRIST_PWM_PER_DEGREE = 750.0 / 180.0; //todo if we need it
    public static double TURRET_PWM_PER_DEGREE = 16; //todo

    public static double ELBOW_SPEED = 220;
    public static double SHOULDER_SPEED = 220;
    public static double LASSO_SPEED = 120;
    public static double WRIST_SPEED = 90;

    public static double kF = 0.0;

    //TODO - FIND THESE TWO
    public static double coneStackShoulderAngle = 0;
    public static double coneStackWristAngle = 13;

    public static double coneStackChassisLength = MAX_CHASSIS_LENGTH- 20;
    public static double WRIST_HOME_POSITION = 31;

    public static double SHOULDER_DEG_MIN = -110; // negative angles are counter clockwise while looking at the left side
    public static double SHOULDER_DEG_MAX = 110; // of the robot

    public static double ELBOW_DEG_MIN = -150;
    public static double WRIST_DEG_MIN = -180;
    public static double TURRET_DEG_MIN = -360;

    public static double ELBOW_DEG_MAX = 150;
    public static double WRIST_DEG_MAX = 180;
    public static double TURRET_DEG_MAX = 360;

    public static double GRIPPER_CLOSED = 1700; //1750 for Leo's gripper

    public static double GRIPPER_RELEASE = 1000;
    public static double GRIPPER_OPEN = 1000; //1100 for Leo's gripper
    public static double TRANSFER_CHASSIS_SHORTEN_BY = 4.5;
    public static double TRANSFER_SHOULDER_ANGLE = -15;
    public static double TRANSFER_SHOULDER_APPROACH_ANGLE = 10;

    public static double TRANSFER_SHOULDER_OUTOFWAY_ANGLE = 20;
    public static double TRANSFER_ELBOW_ANGLE = -115;
    public static double TRANSFER_ELBOW_ANGLE_OUTOFWAY_ANGLE = -135;

    public static double TRANSFER_WRIST_ANGLE = 115;

    public static double TRANSFER_TURRET_ANGLE = 5; //-14


    public static double PICKUP_WRIST_ANGLE = -5;

    public static double PICKUP_SHOULDER_ANGLE  = 70;

    public static double CANCEL_TRANSFER_TURRET_DEGREES = 30;//todo not any real number can be calibrated if necessary

    public static double CANCEL_TRANSFER_SHOULDER = 64 , CANCEL_TRANSFER_ELBOW = 108, CANCEL_TRANSFER_WRIST = -4;
    boolean lassoGripped = false;

    public static double UNDERARM_HEIGHT = 7; //height of rotation of shoulder motor   INCHES

    //these 5 config variables won't have any effect if changed in dashboard since they are currently assigned at compile

    public Servo elbowServo, wristServo, lassoServo;
    public Servo shoulderServo, turretServo;
    public Joint lasso, shoulder, elbow, wrist;
    public boolean enableLasso;

    private double chariotDistance;
    private double shoulderAngle;
    private double shoulderTargetAngle, elbowTargetAngle, wristTargetAngle, turretTargetAngle;

    private Articulation articulation;

    public UnderArm(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);

        if (simulated) { //if (simulated) { ignoring simulation as a way to test real underarm off robot
//            shoulderServo = new ServoSim();
//            elbowServo = new ServoSim();
            wristServo = new ServoSim();
            lassoServo = new ServoSim();
            turretServo = new ServoSim();
        } else {
//            shoulderServo = hardwareMap.get(ServoImplEx.class, "shoulderServo");
//            ((ServoImplEx) shoulderServo).setPwmRange(axonRange);
//            elbowServo = hardwareMap.get(ServoImplEx.class, "elbowServo");
//            ((ServoImplEx) elbowServo).setPwmRange(axonRange);
            wristServo = hardwareMap.get(ServoImplEx.class, "wristServo");
            turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
            ((ServoImplEx) turretServo).setPwmRange(axonRange);
            elbow = new Joint(hardwareMap, "elbowJoint", simulated, ELBOW_HOME_PWM, ELBOW_PWM_PER_DEGREE, ELBOW_DEG_MIN, ELBOW_DEG_MAX, 0, ELBOW_SPEED);
            shoulder = new Joint(hardwareMap, "shoulderJoint", simulated, SHOULDER_HOME_PWM, SHOULDER_PWM_PER_DEGREE, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX, 0, SHOULDER_SPEED);
            wrist = new Joint(hardwareMap, "wristServo", simulated, WRIST_HOME_PWM, WRIST_PWM_PER_DEGREE, WRIST_DEG_MIN, WRIST_DEG_MAX, 0, WRIST_SPEED);
            lassoServo = hardwareMap.get(ServoImplEx.class, "lassoJoint");
            ((ServoImplEx) lassoServo).setPwmRange(axonRange);
        }

        shoulderTargetAngle = 0;
        elbowTargetAngle = 0;
        wristTargetAngle = 0;
        turretTargetAngle = 0;

        articulation = Articulation.init1;
        fieldPositionTarget = new Vector3(0,0,0); //crane default IK starting point is

        setWristTargetAngle(WRIST_HOME_POSITION);
    }

    Vector3 underArmPosition;

    public Vector3 getUnderArmPosition(){
        return underArmPosition;        //returns the calculated underarm position
    }

    Vector3 fieldPositionTarget;

    public enum Articulation {
        init1,
        transfer,
        driving,
        transferRecover, //recover from the transfer position to just outward of the vertical position so gripper clears over the camera
        home,
        manual,
        fold,
        safe,
        unfold,
        jointAngles,
        substationHover,
        coneStackHover,
        coneStackPickup,

        cancelTransferPosition,
        substationPickup,
        substationRecover
    }

    private JointAngle jointAngle;

    public enum JointAngle{
        Home(0,0,0,0,MAX_CHASSIS_LENGTH),
        Test1(90,0,0),
        Test2(90,90,0),
        Test3(0,90,0),
        Test4(90,0,30),
        SafePos(0,0,0,0,MIN_CHASSIS_LENGTH),
        FoldPosition(FOLDPOS_SHOULDER_ANGLE,FOLDPOS_ELBOW_ANGLE, 0, FOLDPOS_TURRET_ANGLE,MIN_CHASSIS_LENGTH),
        UnfoldPosition(20,0,0,0,MAX_CHASSIS_LENGTH);

        public double shoulderAngle, elbowAngle, wristAngle, turretAngle, chassisLength;

        JointAngle(double shoulder, double elbow, double wrist, double turret, double length){
            this.shoulderAngle = shoulder;
            this.elbowAngle = elbow;
            this.wristAngle = wrist;
            this.turretAngle = turret;
            this.chassisLength = length;
        }

        JointAngle(double shoulder, double elbow, double turret){
            this.shoulderAngle = shoulder;
            this.elbowAngle = elbow;
            this.turretAngle = turret;
        }
    }

    public void setJointAngle(JointAngle angle){
        jointAngle = angle;
    }

    public static double FOLDPOS_SHOULDER_ANGLE = -70;
    public static double FOLDPOS_ELBOW_ANGLE = 0;
    public static double FOLDPOS_TURRET_ANGLE = 0;

    public Articulation articulate(Articulation target){
        articulation = target;

        switch(articulation){
            case init1:
                //TODO - FIND TOP CONE WRIST AND SHOULDER
                coneStackWristAngle = 31;
                coneStackShoulderAngle = 0;
                setShoulderTargetAngle(0);
                setElbowTargetAngle(FOLDPOS_ELBOW_ANGLE);
                setTurretTargetAngle(FOLDPOS_TURRET_ANGLE);
                setWristTargetAngle(0);
                break;
            case coneStackHover:
                //tune angles to reach cone stack height
                coneStackHover();
                break;
            case fold:
                /*
                jointAngle = JointAngle.FoldPosition;
                goToJointAngleAndLength(jointAngle);

                 */
                setShoulderTargetAngle(FOLDPOS_SHOULDER_ANGLE);
                setElbowTargetAngle(FOLDPOS_ELBOW_ANGLE);
                setTurretTargetAngle(FOLDPOS_TURRET_ANGLE);
                robot.driveTrain.articulate(DriveTrain.Articulation.lock);
                break;
            case unfold: //PROB NOT NEEDED but added just in case
                jointAngle = JointAngle.UnfoldPosition;
                goToJointAngle(jointAngle);
                articulation = Articulation.manual;
                break;
            case transfer: //transfer position
                if(goToTransfer()){
                    articulation = Articulation.transferRecover;
                }
                break;
            case transferRecover: //after transfer, go back through to clear the camera
                if(TransferRecover())
                    articulation = Articulation.home;
                break;
            case cancelTransferPosition: {
                if(cancelTransferPosition())
                    articulation = Articulation.manual;
                break;
            }
            case safe:
                jointAngle = JointAngle.SafePos;
                goToJointAngleAndLength(jointAngle);
                break;
            case home: //safe position for transiting the field
                if(goHome()){
                    articulation = Articulation.manual;
                }
                break;
            case driving:
                setShoulderTargetAngle(20);
                setElbowTargetAngle(0);
                setTurretTargetAngle(0);
                robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH);
            case manual: //normal IK intake mode
                //holdTarget(fieldPositionTarget.x,fieldPositionTarget.y,fieldPositionTarget.z);
                break;
            case coneStackPickup:
                if(coneStackPickup())
                    articulation = Articulation.manual;
                break;
            case jointAngles:
                goToJointAngle(jointAngle);
            case substationHover:
//                robot.driveTrain.articulate(DriveTrain.Articulation.lock);
                if (goSubstationHover()) articulation = Articulation.manual;
                break;
            case substationPickup:
                if (SubstationPickup()) articulation = Articulation.manual;
                break;

            default:

        }
        return target;
    }

    public void goToJointAngle(JointAngle joint){
        setShoulderTargetAngle(joint.shoulderAngle);
        setElbowTargetAngle(joint.elbowAngle);
        setTurretTargetAngle(joint.turretAngle);
    }

    public void goToJointAngleAndLength(JointAngle joint){
        setShoulderTargetAngle(joint.shoulderAngle);
        setElbowTargetAngle(joint.elbowAngle);
        setTurretTargetAngle(joint.turretAngle);
        robot.driveTrain.setChassisLength(joint.chassisLength);
    }

    public void holdTarget(double x, double y, double z){
        calculateFieldTargeting(x,y,z);
        goToCalculatedTarget();
    }

    public static double speedBoost = 1.5;
    public void speedMode(){
        elbow.setSpeed(ELBOW_SPEED*speedBoost);
        shoulder.setSpeed(SHOULDER_SPEED*speedBoost);
    }

    public void regularMode(){
        elbow.setSpeed(ELBOW_SPEED);
        shoulder.setSpeed(SHOULDER_SPEED);
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

    public boolean atConeStack(){
        return atConeStack;
    }
    public void coneStackHoverStageAdvance() {
        coneStackHoverStage  ++;
    }

    int coneStackHoverStage = 0;
    long coneStackHoverTimer = 0;
    boolean atConeStack = false;

    public boolean coneStackHover() {
        switch (coneStackHoverStage) {
            case 0:
                atConeStack = false;
                robot.driveTrain.setChassisLength(coneStackChassisLength-7);
                coneStackHoverTimer = futureTime(0.5);
//                coneStackHoverStage++;
                break;
            case 1:
                if(System.nanoTime() > coneStackHoverTimer) {
                    setShoulderTargetAngle(coneStackShoulderAngle);
                    setWristTargetAngle(coneStackWristAngle);
                    setElbowTargetAngle(SS_HOVER_ELBOW);
                    coneStackHoverTimer = futureTime(0.5);
//                    coneStackHoverStage++;
                }
            case 2:
                if(System.nanoTime() > coneStackHoverTimer) {
                    robot.driveTrain.setChassisLength(coneStackChassisLength);
//                    coneStackHoverStage++;
                }
                break;
            case 3:
                atConeStack = true;
                coneStackHoverStage = 0;
                return true;
        }
        return false;
    }

    int coneStackPickupStage = 0;
    long coneStackPickupTimer = 0;

    public void coneStackPickupStageAdvance() {
        coneStackPickupStage ++;
    }
    public boolean coneStackPickup() {
        switch (coneStackPickupStage) {
            case 0:
                wrist.setTargetAngle(coneStackWristAngle - 20);
                shoulder.setTargetAngle(coneStackShoulderAngle - 10);
                coneStackPickupTimer = futureTime(.5);
//                coneStackPickupStage ++;
                break;
            case 1:
                robot.driveTrain.setChassisLength(coneStackChassisLength - 7);
//                coneStackPickupStage ++;
                break;
            case 2:
                coneStackPickupStage = 0;
                return true;
        }
        return false;
    }
    long homeNoTuckTimer = 0;
    int homeNoTuckStage = 0;

    public boolean HomeNoTuck () {
        switch (homeNoTuckStage) {
            case 0: //sets home position
                setTurretTargetAngle(0);
                setElbowTargetAngle(0);
                setShoulderTargetAngle(0);
                grip();
                setWristTargetAngle(WRIST_HOME_POSITION);
                homeNoTuckTimer= futureTime(0.5);
                homeNoTuckStage++;
                break;
            case 1:
                if (System.nanoTime() > homeNoTuckTimer) {
                    homeNoTuckStage = 0;
                    return true;
                }
                break;
        }

        return false;

    }

    long cancelTransferPositionTimer = 0;
    int cancelTransferPositionStage = 0;

    public boolean cancelTransferPosition() {
        switch (cancelTransferPositionStage) {
            case 0: //sets home position
                setTurretTargetAngle(0);
                setElbowTargetAngle(0);
                setShoulderTargetAngle(0);
                cancelTransferPositionTimer = futureTime(0.5);
                cancelTransferPositionStage++;
                break;
            case 1: //give the elbow a head start to clear the camera when a cone is loaded
                if (cancelTransferPositionTimer<System.nanoTime()){
                    setElbowTargetAngle(CANCEL_TRANSFER_ELBOW);
                    cancelTransferPositionTimer = futureTime(0.25);
                    cancelTransferPositionStage++;
                }
                break;
            case 2:
                if (cancelTransferPositionTimer < System.nanoTime()) {
                    setElbowTargetAngle(CANCEL_TRANSFER_ELBOW);
                    setShoulderTargetAngle(CANCEL_TRANSFER_SHOULDER);
                    setWristTargetAngle(CANCEL_TRANSFER_WRIST);
                    setTurretTargetAngle(CANCEL_TRANSFER_TURRET_DEGREES);
                    release();
                    cancelTransferPositionTimer = futureTime(0.5);
                    cancelTransferPositionStage++;
                }
                break;
            case 3:
                if (System.nanoTime() > cancelTransferPositionTimer) {
                    cancelTransferPositionStage = 0;
                    return true;
                }
                break;
        }

        return false;

    }

    long homeTimer;
    int homeStage = 0;

    public boolean goHome(){

        switch (homeStage) {
            case 0: //sets home position
                robot.driveTrain.articulate(DriveTrain.Articulation.unlock);
                grip();
                setTurretTargetAngle(0);
                setElbowTargetAngle(0);
                setShoulderTargetAngle(0);
                setWristTargetAngle(WRIST_HOME_POSITION);
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
            case 0: //sets the approach position that gets the cone over the transfer plate
                atTransfer = false;
                setTurretTargetAngle(TRANSFER_TURRET_ANGLE);
                setElbowTargetAngle(TRANSFER_ELBOW_ANGLE);
                setShoulderTargetAngle(TRANSFER_SHOULDER_APPROACH_ANGLE);
                setWristTargetAngle(TRANSFER_WRIST_ANGLE);
                transferTimer = futureTime(0.5);
                transferStage++;
                break;
            case 1: //get closer to Crane so we are over the transfer plate and then drop shoulder
                if(System.nanoTime() > transferTimer){
                    robot.driveTrain.setChassisLength(MAX_CHASSIS_LENGTH-TRANSFER_CHASSIS_SHORTEN_BY);
                    setShoulderTargetAngle(TRANSFER_SHOULDER_ANGLE);
                    transferTimer = futureTime(_TuneableTimer); //todo set real
                    transferStage++;
                }
                break;
            case 2: //release the cone
                if(System.nanoTime() > transferTimer){
                    release();
                    transferTimer = futureTime(0.3);
                    transferStage++;
                }
                break;
            case 3: //lift elbow and gripper away from cone
                if(System.nanoTime() > transferTimer){
                    setElbowTargetAngle(TRANSFER_ELBOW_ANGLE_OUTOFWAY_ANGLE);
                    transferTimer = futureTime(0.3);
                    transferStage++;
                }
                break;
            case 4: //raise underarm further out of the way && close gripper
                if(System.nanoTime() > transferTimer){
                    setShoulderTargetAngle(TRANSFER_SHOULDER_OUTOFWAY_ANGLE);
                    grip();
                    transferTimer = futureTime(0.3);
                    transferStage++;
                }
                break;
            case 5: //set wrist angle for passthrough
                if(System.nanoTime() > transferTimer){
                    grip(); //redundant grip, just to be sure
                    setWristTargetAngle(WRIST_HOME_POSITION);
                    transferStage++;
                }
                break;
            case 6:
                transferStage = 0;
                atTransfer = true;
                return true;

        }

        return false;
    }


    public static double POSTTRANSFER_SHOULDER = 50;
    public static double POSTTRANSFER_ELBOW = -15;

    int transferRecoverStage;
    long transferRecoverTimer;
    public boolean TransferRecover(){
        switch (transferRecoverStage) {
            case 0: //starting from the way up transfer cleared position, recover the elbow first
                atTransfer = false;
                WRIST_SPEED = 270;
                setElbowTargetAngle(POSTTRANSFER_ELBOW);
                setShoulderTargetAngle(POSTTRANSFER_SHOULDER);
                transferRecoverTimer = futureTime(0.5);
                transferRecoverStage++;
                break;
            case 1:
                if(System.nanoTime() > transferRecoverTimer) {
                    transferRecoverTimer = futureTime(0.2);
                    setWristTargetAngle(40); //take wrist homeish
                    transferRecoverStage++;
                }
                break;
            case 2: //lift shoulder
                if (System.nanoTime() > transferRecoverTimer) {
                    setTurretTargetAngle(0); //straighten turret
                    setShoulderTargetAngle(0); //take the shoulder home
                    setWristTargetAngle(40); //take wrist homeish
                    robot.driveTrain.setChassisLength(MAX_CHASSIS_LENGTH-10);
                    transferTimer = futureTime(0.3);
                    transferRecoverStage++;
                }
                break;
            case 3:
                if(System.nanoTime() > transferRecoverTimer) {
                    transferRecoverStage = 0;
                    return true;
                }
                break;
        }

        return false;
    }

    public boolean atTransfer(){
        return atTransfer;
    }

    long substationHoverTimer;
    int substationHoverStage = 0;

    public double SS_HOVER_SHOULDER = 64 , SS_HOVER_ELBOW = 108, SS_HOVER_WRIST = 60, SS_HOVER_TURRET = 0, SS_HOVER_EXTEND = MIN_SAFE_CHASSIS_LENGTH;

    boolean canSaveHoverPositions = false;
    public void SaveHoverPositions(){
        if(canSaveHoverPositions) {
            //only call this when at a good known substation hover position
            SS_HOVER_ELBOW = elbowTargetAngle;
            SS_HOVER_SHOULDER = shoulderTargetAngle;
            SS_HOVER_TURRET = turretTargetAngle;
            SS_HOVER_EXTEND = robot.driveTrain.getTargetChassisLength();
            canSaveHoverPositions = false;
        }
    }

    public void SetDefaultHoverPositions(){
        SS_HOVER_SHOULDER = 57; SS_HOVER_ELBOW = 108; SS_HOVER_WRIST = 68; SS_HOVER_TURRET = 0;
    }
    public boolean goSubstationHover() {
        switch (substationHoverStage) {
            case 0:
                if (goSubstationRecover()) { //give the elbow a head start to clear the camera when a cone is loaded
                    robot.driveTrain.setChassisLength(SS_HOVER_EXTEND);
                    setElbowTargetAngle(SS_HOVER_ELBOW);
                    grip(); //make sure the gripper is closed in case we are returning from negative angles
                    substationHoverTimer = futureTime(0.1);
                    substationHoverStage++;
                }
                break;
            case 1:
                if (substationHoverTimer<System.nanoTime()){
                    setShoulderTargetAngle(SS_HOVER_SHOULDER/1.6); //mostly go there but stop early to not slam the ground
                    open();
                    substationHoverTimer = futureTime(0.4);
                    substationHoverStage++;
                }
                break;
            case 2:

                if (substationHoverTimer<System.nanoTime()){
                    // these values are meant to be fine tuned by driver positioning between hover and pickup
                    setElbowTargetAngle(SS_HOVER_ELBOW);
                    setShoulderTargetAngle(SS_HOVER_SHOULDER);
                    setWristTargetAngle(PICKUP_WRIST_ANGLE);
                    setTurretTargetAngle(SS_HOVER_TURRET);
                    substationHoverTimer = futureTime(0.5);
                    substationHoverStage++;
                }
                break;
            case 3: //open gripper
                if (substationHoverTimer<System.nanoTime()) {
                    open(); //purposefully redundant open
                    substationHoverStage = 0;
                    canSaveHoverPositions = true;
                    return true;
                }
                break;
        }
        return false;
    }

    int substationPickupStage = 0;
    long substationPickupTimer = 0;

    public boolean SubstationPickup() {
        switch (substationPickupStage) {
            case 0: //rotate wrist down to horizontal
                robot.underarm.SaveHoverPositions();
                WRIST_SPEED = 270;
                setWristTargetAngle(PICKUP_WRIST_ANGLE);
                substationPickupTimer = futureTime(0.3);
                substationPickupStage++;

                break;
            case 1: //bring lasso to mat with shoulder only
                if (substationPickupTimer<System.nanoTime()) {
                    WRIST_SPEED = 90;
                    robot.driveTrain.setChassisLength(SS_HOVER_EXTEND+7);
                    setShoulderTargetAngle(PICKUP_SHOULDER_ANGLE);
                    substationPickupStage++;
                    substationPickupTimer = futureTime(0.3); //0.3 todo this is set long from manual cone placement in testing
                }
                break;

            case 2: //close lasso
                if (substationPickupTimer<System.nanoTime()) {
                    grip();
                    substationPickupStage++;
                    substationPickupTimer = futureTime(0.2);
                }
                break;
            case 3: //elevate shoulder to clear mat
                if (substationPickupTimer<System.nanoTime()) {
                    setWristTargetAngle(WRIST_HOME_POSITION);
                    substationPickupStage++;
                    substationPickupTimer = futureTime(0.1);
                }
                break;
            case 4:
                if(substationPickupTimer<System.nanoTime()){
                    setShoulderTargetAngle(SS_HOVER_SHOULDER);
                    substationPickupStage++;
                    substationPickupTimer = futureTime(0.3);
                }
                break;
            case 5: //recover
                if (substationPickupTimer<System.nanoTime()){
                    if( goSubstationRecover()) {
                        substationPickupStage = 0;
                        return true;
                    }
                }
                break;
            default:
                return false;
        }
        return false;
    }

    //this is ugly, but a way to cleanup stages in interrupted underarm articulations
    public void resetArticulations(){
        recoverStage = 0;
        substationHoverStage = 0;
        substationPickupStage = 0;
        transferStage = 0;
        transferRecoverStage = 0;
        homeStage = 0;
    }
    int recoverStage = 0;
    long recoverTimer = 0;



    /**
     *  Recover from picking up a cone to a nearly vertical position
     *  Not the same as home because it doesn't change the chassis length
     * @return true means the behavior is complete
     */
public boolean goSubstationRecover() {
            switch (recoverStage) {
        case 0: //sets vertical position
            robot.driveTrain.articulate(DriveTrain.Articulation.unlock);
            setElbowTargetAngle(0);
            setShoulderTargetAngle(0);
            setWristTargetAngle(WRIST_HOME_POSITION);
            recoverTimer = futureTime(0.5);
            recoverStage++;
            break;
        case 1:
            if (System.nanoTime() > recoverTimer) {
                setTurretTargetAngle(0);
                recoverStage = 0;
                return true;
            }
            break;
    }
        return false;
}

    double calculatedTurretAngle;
    double calculatedHeight;
    double calculatedDistance;
    double calculatedShoulderAngle;
    double calculatedElbowAngle;

    public static double shoulderHeight = 9;

    public static double UPPER_ARM_LENGTH = 14.06;
    public static double LOWER_ARM_LENGTH = 11.02;

    public double getCalculatedShoulderAngle(){
        return calculatedShoulderAngle;
    }

    public double getCalculatedHeight(){
        return calculatedHeight;
    }

    public double getCalculatedDistance(){
        return calculatedDistance;
    }

    public boolean calculateFieldTargeting(double x, double y, double z){ //THIS IS IN INCHES!!!!!!!!

        z /= INCHES_PER_METER;

        double hypotToTarget;
        double locX;
        double locY;
//        double calculatedDistanceFieldUnits;

        locX = x - underArmPosition.x;
        locY = y - underArmPosition.y;


        calculatedTurretAngle = Math.toDegrees(robot.driveTrain.gridHeading) - Math.toDegrees(Math.atan2(locY, locX));

        calculatedHeight = z-shoulderHeight;

        calculatedDistance = (Math.sqrt(Math.pow(locY,2) + Math.pow(locX,2)));

//        calculatedDistanceFieldUnits = (Math.sqrt(Math.pow(locY,2) + Math.pow(locX,2)));

        hypotToTarget = Math.sqrt(Math.pow(calculatedHeight, 2) + Math.pow(calculatedDistance, 2));

//        double virtualLength = Math.sqrt( Math.pow(calculatedDistance,2) + Math.pow(calculatedHeight,2) );

//        calculatedElbowAngle = Math.toDegrees(2*Math.asin( virtualLength / (SHOULDER_TO_ELBOW + ELBOW_TO_WRIST) ));
//        calculatedShoulderAngle = Math.toDegrees(Math.atan2( calculatedDistance , calculatedHeight ) + (90 - calculatedElbowAngle/2));

        //todo: hope and pray that this works
        calculatedShoulderAngle = 90 - (Math.toDegrees(Math.acos(
                (Math.pow(UPPER_ARM_LENGTH, 2) + Math.pow(calculatedDistance, 2) + Math.pow(calculatedHeight, 2) - Math.pow(LOWER_ARM_LENGTH, 2))
                                        / (2 * UPPER_ARM_LENGTH * hypotToTarget))
                                        + Math.acos(calculatedDistance / hypotToTarget)));

        calculatedElbowAngle = (Math.toDegrees(Math.acos(
                                (Math.pow(UPPER_ARM_LENGTH, 2) + Math.pow(LOWER_ARM_LENGTH, 2) - Math.pow(calculatedDistance, 2) - Math.pow(calculatedHeight , 2))
                                                            / (2 * UPPER_ARM_LENGTH * LOWER_ARM_LENGTH))));

        return true;
    }

    //todo these adjust methods are horrid - they need to have range limits applied to them and time based velocity if we are keeping them

    public void adjustShoulder(double speed){
        shoulderTargetAngle -= ADJUST_SHOULDER*speed*robot.deltaTime;
        shoulderTargetAngle = Range.clip(shoulderTargetAngle, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
    }

    public void adjustElbow(double speed){
        elbowTargetAngle += ADJUST_ELBOW*speed*robot.deltaTime;
        elbowTargetAngle = Range.clip(elbowTargetAngle, ELBOW_DEG_MIN, ELBOW_DEG_MAX);

    }

    public void adjustWrist(double speed){
        wristTargetAngle += ADJUST_WRIST*speed*robot.deltaTime;
        wristTargetAngle = Range.clip(wristTargetAngle, WRIST_DEG_MIN, WRIST_DEG_MAX);
    }

    public void adjustTurret(double speed){
        turretTargetAngle += ADJUST_TURRET*speed*robot.deltaTime;
        turretTargetAngle = Range.clip(turretTargetAngle, TURRET_DEG_MIN, TURRET_DEG_MAX);

    }

    public double getShoulderAngle(){
        return shoulderTargetAngle;
    }

    public double getElbowAngle(){
        return elbowTargetAngle;
    }

    public static double ADJUST_TURRET = 50;
    public static double ADJUST_ELBOW = 50;
    public static double ADJUST_SHOULDER = 50;
    public static double ADJUST_WRIST = 50;

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

        elbow.setPWM_PER_DEGREE(ELBOW_PWM_PER_DEGREE);
        shoulder.setPWM_PER_DEGREE(SHOULDER_PWM_PER_DEGREE);
        wrist.setPWM_PER_DEGREE(WRIST_PWM_PER_DEGREE);

        Pose2d robotPosInches = robot.driveTrain.getPoseEstimate();
        double headingRad = robot.driveTrain.getRawHeading();
        double underArmLengthInches = robot.driveTrain.getChassisLength();

        underArmPosition =  new Vector3(robotPosInches.getX()+underArmLengthInches*Math.cos(headingRad),
                robotPosInches.getY()+underArmLengthInches*Math.sin(headingRad),
                UNDERARM_HEIGHT); //calculates and sets the position of underarm in world coordinates

        chariotDistance = robot.driveTrain.getChassisLength();

        articulate(articulation);

        if(elbowTargetAngle > 150){
            elbowTargetAngle = 150;
        }else if(elbowTargetAngle < -150){
            elbowTargetAngle = -150;
        }

        if(shoulderTargetAngle > 100){
            shoulderTargetAngle = 100;
        }else if(shoulderTargetAngle < -100){
            shoulderTargetAngle = -100;
        }

        shoulder.setTargetAngle(shoulderTargetAngle);
        elbow.setTargetAngle(elbowTargetAngle);
        wrist.setTargetAngle(wristTargetAngle -13 ); //the -17.5 is a kludgy way to account for the 35 degree mechanical range shifter - prolly still need to retune each wrist angle
        //wristServo.setPosition(servoNormalizeExtended(wristServoValue(wristTargetAngle)));
        turretServo.setPosition(servoNormalizeExtended(turretServoValue(turretTargetAngle)));

        elbow.update();
        shoulder.update();
        wrist.update();
    }

    public void setEnableLasso(boolean yayornay) {
        enableLasso = yayornay;
    }
    public void grip(){
        lassoGripped = true;
        lassoServo.setPosition(servoNormalize(GRIPPER_CLOSED));
    }

    public void open(){
        lassoGripped = false;
        lassoServo.setPosition(servoNormalize(GRIPPER_OPEN));
    }
    public void release(){ //open wide enough to release the cone but not stress the underarm
        lassoGripped = false;
        lassoServo.setPosition(servoNormalize(GRIPPER_RELEASE));
    }

    public void toggleLasso(){
        if (lassoGripped) {
            open();
        } else {
            grip();
        }
    }

    //MAKE IT RESET ON INIT
    //angles are in degrees
    public void updateConeStackAngles() {
        coneStackShoulderAngle -= Constants.CONESTACK_SHOULDER_ADJ_PER_CONE * robot.field.getConeStack(robot.getAutonConeStack()).getConeNum();
        coneStackWristAngle += Constants.CONESTACK_WRIST_ADJ_PER_CONE * robot.field.getConeStack(robot.getAutonConeStack()).getConeNum();
    }

    @Override
    public void stop() {
        shoulder.RelaxJoint();
        elbow.RelaxJoint();

    }

    @Override
    public String getTelemetryName() {
        return "UnderArm";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);
        telemetryMap.put("Transfer Stage", transferStage);

        if (debug) {
            telemetryMap.put("Shoulder Target Angle", shoulderTargetAngle);
            telemetryMap.put("Elbow Target Angle", elbowTargetAngle);
            telemetryMap.put("Wrist Target Angle", wristTargetAngle);
            telemetryMap.put("Turret Target Angle", turretTargetAngle);
            telemetryMap.put("Calc Elbow", calculatedElbowAngle);
            telemetryMap.put("Calc Shoulder", calculatedShoulderAngle);
            telemetryMap.put("Under Arm Pos X", underArmPosition.x);
            telemetryMap.put("Under Arm Pos Y", underArmPosition.y);
            telemetryMap.put("Under Arm Target X", fieldPositionTarget.x);
            telemetryMap.put("Under Arm Target Y", fieldPositionTarget.y);
            telemetryMap.put("Under Arm Target Z", fieldPositionTarget.z);

            telemetryMap.put("Elbow Target PWM", elbowServoValue(elbowTargetAngle));
            telemetryMap.put("Shoulder Target PWM", shoulderServoValue(shoulderTargetAngle));
            telemetryMap.put("Wrist Target PWM", wristServoValue(wristTargetAngle));
            telemetryMap.put("Turret Target PWM", turretServoValue(turretTargetAngle));
            telemetryMap.put("Lasso Position", lassoServo.getPosition());
            telemetryMap.put("Lasso Grip", lassoGripped);
            telemetryMap.put("Home Stage", homeStage);

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
        this.shoulderTargetAngle = shoulderTargetAngle;
    }

    public void setElbowTargetAngle(double elbowTargetAngle) {
        this.elbowTargetAngle = elbowTargetAngle;
    }

    public void setWristTargetAngle(double wristTargetAngle) {
        this.wristTargetAngle = wristTargetAngle;
    }

    public void setTurretTargetAngle(double turretTargetAngle) {
        this.turretTargetAngle =turretTargetAngle;
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
