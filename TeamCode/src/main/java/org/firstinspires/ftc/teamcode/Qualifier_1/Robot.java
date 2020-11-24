/**
 * Aamod, i appoint you the owner of this file.
 *
 * @author: Aamod
 * @version: 1.0
 * @status: work in progress
 */

package org.firstinspires.ftc.teamcode.Qualifier_1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.RingDepositor;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Intake;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.IMUChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.OdometryChassis;
//import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.VuforiaWebcam;


public class Robot {
//    Shooter shooter = new Shooter();
//    WobbleGoal wobbleGoal = new WobbleGoal();

    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    private ElapsedTime runtime = null;
    final boolean isCorgi = Chassis.isCorgi;

    // Hardware Objects
    private BasicChassis drivetrain = null;
    private Intake intake = null;
    private WobbleGoal wobbleGoal = null;
    private RingDepositor ringDepositor = null;
    private Shooter shooter = null;
    public VuforiaWebcam vuforiaWebcam = null;
    public TensorFlow tensorFlow = null;

    private double vuforiaX = 0;
    private double vuforiaY = 0;
    private double vuforiaAngle = 0;
    private double robotAngle = 0;

    public Robot(LinearOpMode opMode, BasicChassis.ChassisType chassisType ) {
        op = opMode;
        hardwareMap = op.hardwareMap;
        WobbleGoal.Position currentWobbleGoalPosition = WobbleGoal.Position.REST;
        RingDepositor.Position currentRingDepositorPosition = RingDepositor.Position.REST;
        runtime = new ElapsedTime();
        if(chassisType==BasicChassis.ChassisType.ENCODER){
            drivetrain = new EncoderChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.IMU){
            drivetrain = new IMUChassis(op);
        }
        else if(chassisType==BasicChassis.ChassisType.ODOMETRY){
            drivetrain = new OdometryChassis(op);
        }
        if(!isCorgi){
            vuforiaWebcam = new VuforiaWebcam(op);
            tensorFlow = new TensorFlow(op);
        }
        if(isCorgi) {
            intake = new Intake(op);
            wobbleGoal = new WobbleGoal(op);
            ringDepositor = new RingDepositor(op);
            intake = new Intake(op);
            shooter = new Shooter(op);
        }

        // comment by victor
        // drivetrain.init(opMode);
//        if(!isCorgi) {
//            //vuforiaWebcam.init(opMode); //conflicts with TensorFlow
//        }
    }
    public void moveVuforiaWebcam(double x, double y, double endAngle) {
        if(!isCorgi) {
            getVuforiaPosition();

            double xdifference = x - vuforiaX;
            double ydifference = y - vuforiaY;

            double turn = endAngle - robotAngle;

            double magnitude = Math.sqrt((xdifference * xdifference) + (ydifference * ydifference));

            double mAngle = robotAngle - Math.toDegrees(Math.acos(ydifference / magnitude)); //move Angle

            op.telemetry.addData("VuforiaX", "%.2f %.2f %.2f %.3f %.3f %.3f", vuforiaX, x, xdifference, robotAngle, vuforiaAngle, endAngle);
            op.telemetry.addData("VuforiaY", "%.2f %.2f %.2f %.2f %.3f %.3f", vuforiaY, y, ydifference, magnitude, turn, mAngle);
            op.telemetry.update();
            op.sleep(5000);
            //drivetrain.moveAngle2(magnitude, mAngle, turn);

            getVuforiaPosition();

            op.telemetry.addData("VuforiaX", "%.2f %.2f %.2f %.3f %.3f %.3f", vuforiaX, x, xdifference, robotAngle, vuforiaAngle, endAngle);
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
    public void moveMultidirectional(double power, double angle, float rightStick) {
        drivetrain.moveMultidirectional(power, angle, rightStick);
    }

    /*public void moveAngle2(double distance, double angle, double turn) {
        drivetrain.moveAngle2(distance, angle, turn);
    }*/


    /**Vuforia**/

    public double getVuforiaAngle() {
        if(!isCorgi) {
            return vuforiaWebcam.getVuforiaAngle();
        }
        return 0;
    }

    public void getVuforiaPosition() {
        if(!isCorgi) {
            vuforiaX = vuforiaWebcam.getVuforiaX();
            vuforiaY = vuforiaWebcam.getVuforiaY();
            vuforiaAngle = vuforiaWebcam.getVuforiaAngle();
            robotAngle = vuforiaAngle + 90;
            robotAngle = (robotAngle > 180 ? robotAngle - 360 : robotAngle);
        }
    }
    public void stopVuforiaTF() {
        if(!isCorgi) {
            vuforiaWebcam.interrupt();
        }
    }

    /**TensorFlow**/

    public void initTensorFlow() {
        tensorFlow.initTensorFlow();
    }

    public void runTensorFlow () {
        tensorFlow.runTensorFlow();
    }

    public void stopTensorFlow () {
        tensorFlow.stopTensorFlow();
    }

    /**Odometry**/

    /**
     * wobble goal methods
     */
    public WobbleGoal.Position wobbleGoalGoToPosition(WobbleGoal.Position p){
        if(isCorgi) {
            wobbleGoal.goToPosition(p);
        }
        return (p);
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
    public void ringDepositorGoToPosition(RingDepositor.Position p){
        if(isCorgi) {
            ringDepositor.goToPosition(p);
        }
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

    public void shootHighGoal(int rings) {
        shooter.shootHighGoal(rings);
    }

    public void shootMidGoal(int distance) {
        shooter.shootMidGoal(distance);
    }

    public void shootLowGoal(int distance) {
        shooter.shootLowGoal(distance);
    }

    public void moveServo(boolean direction) {
        shooter.moveServo(direction);
    }

    public void shootGoalTeleop(int direction, int power) {
        shooter.shootGoalTeleop(direction, power);
    }
}