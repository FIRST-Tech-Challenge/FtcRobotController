package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.Intake;
import org.firstinspires.ftc.teamcode.Components.Accesories.RingDepositor;
import org.firstinspires.ftc.teamcode.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.ChassisFactory;
import org.firstinspires.ftc.teamcode.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Components.ObjectDetection.TensorFlow;

public class Robot {

    private LinearOpMode op = null;
    final boolean isCorgi = true;

    // Hardware Objects
    private BasicChassis drivetrain = null;
    private Intake intake = null;
    private WobbleGoal wobbleGoal = null;
    private RingDepositor ringDepositor = null;
    private Shooter shooter = null;
    private VuforiaWebcam vuforiaWebcam = null;
    private TensorFlow tensorFlow = null;

    private double vuforiaX = 0;
    private double vuforiaY = 0;
    private double robotAngle = 0; //robot angle using vuforia

    public Robot(LinearOpMode opMode, BasicChassis.ChassisType chassisType, boolean objectDetectionNeeded, boolean vuforiaNAVIGATIONneeded ) {
        op = opMode;
        //This link has a easy to understand explanation. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
        drivetrain= ChassisFactory.getChassis(chassisType,op);

        vuforiaWebcam = new VuforiaWebcam(op);
        if(objectDetectionNeeded){
            tensorFlow = new TensorFlow(op);
        }
        if(vuforiaNAVIGATIONneeded){
            vuforiaWebcam.init(op);
        }
        if(objectDetectionNeeded && vuforiaNAVIGATIONneeded){
            throw new RuntimeException("They both can't be true.");
        }

        if(isCorgi) {
            intake = new Intake(op);
            wobbleGoal = new WobbleGoal(op);
            ringDepositor = new RingDepositor(op);
            intake = new Intake(op);
            shooter = new Shooter(op);
        }

        if (objectDetectionNeeded) {
            tensorFlow.runTensorFlowWaitForStart();
        }
    }

    public void moveVuforiaWebcam(double x, double y, double endAngle) {
        if(!isCorgi) {
            getVuforiaPosition();

            double xdifference = x - vuforiaX;
            double ydifference = y - vuforiaY;

            double turn = endAngle - robotAngle;

            double magnitude = Math.sqrt((xdifference * xdifference) + (ydifference * ydifference));

            double mAngle = robotAngle - Math.toDegrees(Math.acos(ydifference / magnitude)); //move Angle

            op.telemetry.addData("VuforiaX", "%.2f %.2f %.2f %.3f %.3f %.3f", vuforiaX, x, xdifference, robotAngle, getVuforiaAngle2(), endAngle);
            op.telemetry.addData("VuforiaY", "%.2f %.2f %.2f %.2f %.3f %.3f", vuforiaY, y, ydifference, magnitude, turn, mAngle);
            op.telemetry.update();
            op.sleep(5000);
            //drivetrain.moveAngle2(magnitude, mAngle, turn);

            getVuforiaPosition();

            op.telemetry.addData("VuforiaX", "%.2f %.2f %.2f %.3f %.3f %.3f", vuforiaX, x, xdifference, robotAngle, getVuforiaAngle2(), endAngle);
            op.telemetry.addData("VuforiaY", "%.2f %.2f %.2f %.2f %.3f %.3f", vuforiaY, y, ydifference, magnitude, turn, mAngle);
            op.telemetry.update();
            op.sleep(5000);
        }
    }

    public void startVuforia () {
        vuforiaWebcam.start();
    }

    public void stopAllMotors() {
        drivetrain.stopAllMotors();
    }


    /*/******** Left Front Motor **********/
   /* public void moveMotorLeftFront(double distance) {
        drivetrain.moveMotorLeftFront(distance);
    }

    /******** Right Front Motor **********/
   /* public void moveMotorRightFront(double distance) {
        drivetrain.moveMotorRightFront(distance);
    }

    /******** Left Back Motor **********/
    /*public void moveMotorLeftBack(double distance) {
        drivetrain.moveMotorLeftBack(distance);
    }

    /******** Right Back Motor **********/
    /*public void moveMotorRightBack(double distance) {
        drivetrain.moveMotorRightBack(distance);
    }*/

    /******** shooterMotor **********/
    /*public void moveShooterMotor(int distance, int power) {
        shooter.moveShooterMotor(distance, power);
    }
    public double getAngle() {
        return drivetrain.getAngle();
    }


    /**
     * Directional Movement
     **/
    public void moveForward(double distance, double power) {
        drivetrain.moveForward(distance, power);
    }

    public void moveBackward(double distance, double power) {
        drivetrain.moveBackward(distance, power);
    }

    public void moveRight(double distance, double power) {
        drivetrain.moveRight(distance, power);
    }
    public void moveAngle(double x, double y, double power){
        drivetrain.moveAngle(x,y,power);
    }
    public void moveLeft(double distance, double power) {
        drivetrain.moveLeft(distance, power);
    }
    public void turnInPlace(double target, double power) {
        drivetrain.turnInPlace(target, power);
    }

    public void moveMultidirectional(double power, double angle, float rightStick, boolean isSlow) {
        drivetrain.moveMultidirectional(power, angle, rightStick, isSlow);
    }

    /*public void moveAngle2(double distance, double angle, double turn) {
        drivetrain.moveAngle2(distance, angle, turn);
    }*/

    /**Navigation**/

    public void runNavigation() {
        Thread vuforia = new Thread(new VuforiaWebcam(op));
        Thread odometry = new Thread(new Odometry(op));

        odometry.start();
        vuforia.start();
        //This is where we write the code that uses Vuforia to override Odometry
    }


    /**Vuforia**/

    public double getVuforiaAngle() {
        if(isCorgi) { //TODO: aamod 11/24
            return vuforiaWebcam.getVuforiaAngle();
        }
        return 0;
    }

    public double getVuforiaAngle2() {
        if(isCorgi) {
            return vuforiaWebcam.getVuforiaAngle2();
        }
        return 0;
    }

    public void getVuforiaPosition() {
        if(isCorgi) { //TODO: aamod 11/24
            vuforiaX = vuforiaWebcam.getVuforiaX();
            vuforiaY = vuforiaWebcam.getVuforiaY();
            double vuforiaAngle = vuforiaWebcam.getVuforiaAngle2();
            robotAngle = vuforiaAngle + 90;
            robotAngle = (robotAngle > 180 ? robotAngle - 360 : robotAngle);
        }
    }
    public void stopVuforiaTF() { //TODO: aamod 11/24
        if(isCorgi) {
            vuforiaWebcam.interrupt();
        }
    }

    /**TensorFlow**/

    public void initTensorFlow() { //This needs to be here, because it is used in the TensorFlow test program
        tensorFlow.initTensorFlow();
    }

    //TODO: Aamod, do we need these 3 methods. Can the robot just expose getNumberOfRings, which will runTensorFlow and then getNumberOfRings from TensorFlow?
    public void runTensorFlow () { //This needs to be here, because it is used in the TensorFlow test program
        tensorFlow.runTensorFlow();
    }

    public void stopRingDetection () {
        tensorFlow.stopTensorFlow();
    }

    public int getRingsAndWaitForStart(){
        tensorFlow.runTensorFlowWaitForStart();
        return tensorFlow.getNumberOfRings();
    }

    public int getNumberOfRings() { //This needs to be here, because it is used in the TensorFlow test program
        return tensorFlow.getNumberOfRings();
    }

    /**Odometry**/

    /**
     * wobble goal methods
     */
    //TODO: Siddharth & Aiden, can we start method name with a verb?
    public WobbleGoal.Position wobbleGoalGoToPosition(WobbleGoal.Position p){
        if(isCorgi) {
            wobbleGoal.goToPosition(p);
        }
        return (p);
    }

    //TODO: Siddharth & Aiden, can we be consistent and crate another WobbleGoal.Position for teleop start position?
    public void teleopStartPosition(){
        if (isCorgi) {
            wobbleGoal.teleopStartPosition();
        }
    }

    public void printCurrentWobbleGoalLocation(){
        if(isCorgi) {
            wobbleGoal.printCurrentLocation();
        }
    }

    public void stopWobbleGoal(){
        if(isCorgi) {
            wobbleGoal.stop();
        }
    }

    public void moveWobbleGoalServo(boolean direction){
        wobbleGoal.moveWobbleGoalServo(direction);
    }

    // ring depositor
    //TODO: <owner>, can we start method name with a verb?
    public void ringDepositorGoToPosition(RingDepositor.Position p){
        if(isCorgi) {
            ringDepositor.goToPosition(p);
        }
    }

    //TODO: <owner> , can we start method name with a verb?
    public void ringDepositorSmartDeposit(){
        ringDepositor.smartDeposit();
    }

    public void printCurrentRingDepositorLocation() {
        if(isCorgi) {
            ringDepositor.printCurrentLocation();
        }
    }

    public void stopRingDepositor(){
        if(isCorgi) {
            ringDepositor.stop();
        }
    }

    public void moveRingClamp(boolean direction) {
        if(isCorgi) {
            ringDepositor.moveRingClamp(direction);
        }
    }

    // intake
    public void startIntake(){
        if(isCorgi) {
            intake.startIntake();
        }
    }

    public void stopIntake(){
        if(isCorgi) {
            intake.stopIntake();
        }
    }

    //shooter


    public void moveServo(boolean direction) {
            shooter.moveServo(direction);
    }

    public void shootHighGoal(int rings) {
        shooter.shootHighGoal(rings);
    }

    public void shootMiddleGoal(int rings){
        shooter.shootMidGoal(rings);
    }

    public void shootLowGoal(int rings){
        shooter.shootMidGoal(rings);
    }



    public void shootGoalTeleop(int distance) {
            shooter.shootGoalTeleop(distance);
    }

    public void stopShooter(){
        shooter.stopShooter();
    }












    //TODO: <owner>, why do have the business logic of shooeter motor in Robot? Can this be moved to Shooter class, like some of the other methods?
    public void moveShooterMotor(int distance, int power) {
        double sleepTime = (distance / 1 * 1000);

        shooter.shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooter.shooterMotor.setTargetPosition(distance);

        shooter.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.shooterMotor.setTargetPosition(distance);
        shooter.shooterMotor.setPower(power);
        if (shooter.shooterMotor.getCurrentPosition() == distance) {
            shooter.shooterMotor.setPower(0);
        }


    }

    public void shootLeftPowerShot(int rings) {
        shooter.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.shooterMotor.setVelocity(10000.0);
        shooter.shooterMotor.setPower(1.0);
        op.sleep(1000);
        for(int i=0;i<rings;i++){
            moveServo(false);
            if(i!=rings-1) {
                moveAngle(-10, 0, 0.5);
                turnInPlace(0,1.0);
            }
            moveServo(true);
        }
        shooter.shooterMotor.setPower(0);
    }
    public void shootRightPowerShot(int rings) {
        shooter.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.shooterMotor.setVelocityPIDFCoefficients(38,0,0,17);
        shooter.shooterMotor.setPower(0.950);
        op.sleep(1000);
        for(int i=0;i<rings;i++){
            shooter.shooterMotor.setPower(0.950);
            moveServo(false);
            if(i!=rings-1) {
                moveAngle(9.6, -0.8, 0.5);
                turnInPlace(3,1.0);
            }
            moveServo(true);
        }
        shooter.shooterMotor.setPower(0);
    }




}
