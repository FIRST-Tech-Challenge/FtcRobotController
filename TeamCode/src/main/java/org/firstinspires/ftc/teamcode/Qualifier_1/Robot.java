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
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;
//import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.VuforiaWebcam;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

public class Robot {
//    Shooter shooter = new Shooter();
//    WobbleGoal wobbleGoal = new WobbleGoal();

    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    private ElapsedTime runtime = null;
    final boolean isCorgi = Chassis.isCorgi;

    // Hardware Objects
    private Chassis drivetrain = null;
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

    public Robot(LinearOpMode opMode) {
        op = opMode;
        hardwareMap = op.hardwareMap;

        runtime = new ElapsedTime();
        drivetrain = new Chassis(op);
        if(!isCorgi){ //TODO: fix later
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
    }

    /*
    public Robot() {

    }
     */

    public void initChassis_no_long_in_use (LinearOpMode opMode) {
        op = opMode;
        hardwareMap = op.hardwareMap;

        //Initialize Motors
        DcMotorEx motorLeftFront;
        DcMotorEx motorRightFront;
        DcMotorEx motorLeftBack;
        DcMotorEx motorRightBack;
        if(!isCorgi) {
            vuforiaWebcam.init(opMode);
        }
        DcMotorEx ShooterMotor;
        DcMotorEx wobbleGoalMotor;
        Servo shooter_Servo;


        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        ShooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        shooter_Servo = (Servo) hardwareMap.servo.get("ShooterServo");
        if(!isCorgi) {
            vuforiaWebcam = new VuforiaWebcam(op);
        }
        // comment by Victor
        // drivetrain.init(opMode);
        if(!isCorgi) {
            vuforiaWebcam.init(opMode);
        }

        this.wobbleGoal = new WobbleGoal(op);
        intake = new Intake(op);
        if(!isCorgi) {
            vuforiaWebcam.start();
        }
        shooter=new Shooter(op);
        if(!isCorgi) {
            vuforiaWebcam.start();
            getVuforiaPosition();
        }
        if(!isCorgi) {
            op.telemetry.addData("Position","%.2f %.2f %.2f %.2f", vuforiaX, vuforiaY, vuforiaAngle, robotAngle);
            op.telemetry.update();
            op.sleep(1000);
//            op.telemetry.addData("Position","%.2f %.2f %.2f %.2f", vuforiaX, vuforiaY, vuforiaAngle, robotAngle);
//            op.telemetry.update();
//            op.sleep(1000);
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

            op.telemetry.addData("VuforiaX", "%.2f %.2f %.2f %.3f %.3f %.3f", vuforiaX, x, xdifference, robotAngle, vuforiaAngle, endAngle);
            op.telemetry.addData("VuforiaY", "%.2f %.2f %.2f %.2f %.3f %.3f", vuforiaY, y, ydifference, magnitude, turn, mAngle);
            op.telemetry.update();
            op.sleep(5000);
            drivetrain.moveAngle2(magnitude, mAngle, turn);

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

    /******** Left Front Motor **********/
    public void moveMotorLeftFront(double distance) {
        drivetrain.moveMotorLeftFront(distance);
    }

    /******** Right Front Motor **********/
    public void moveMotorRightFront(double distance) {
        drivetrain.moveMotorRightFront(distance);
    }

    /******** Left Back Motor **********/
    public void moveMotorLeftBack(double distance) {
        drivetrain.moveMotorLeftBack(distance);
    }

    /******** Right Back Motor **********/
    public void moveMotorRightBack(double distance) {
        drivetrain.moveMotorRightBack(distance);
    }

    /******** shooterMotor **********/
    public void moveShooterMotor(int distance, int power) {
        if(isCorgi) {
            drivetrain.moveShooterMotor(distance, power);
        }
    }
    /******** shooterMotor **********/
    public void moveWobbleGoalMotor(double distance) {
        if(isCorgi) {
            drivetrain.moveWobbleGoalMotor(distance);
        }
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

    public void moveForwardIMU(double distance, double power) {
        drivetrain.moveForwardIMU(distance, power);
    }

    public void moveBackward(double distance, double power) {
        drivetrain.moveBackward(distance, power);
    }

    public void moveBackwardIMU(double distance, double power) {
        drivetrain.moveBackwardIMU(distance, power);
    }

    public void moveRight(double distance, double power) {
        drivetrain.moveRight(distance, power);
    }

    public void moveRightIMU(double distance, double power, double startingAngle, double gain, double maxCorrection) {
        drivetrain.moveRightIMU(distance, power, startingAngle, gain, maxCorrection);
    }

    public void moveLeft(double distance, double power) {
        drivetrain.moveLeft(distance, power);
    }

    public void multidirectionalMove(double power, double angle, float rightStick) {
        drivetrain.multidirectionalMove(power, angle, rightStick);
    }

    public void moveLeftIMU(double distance, double power, double startingAngle, double gain, double maxCorrection) {
        drivetrain.moveLeftIMU(distance, power, startingAngle, gain, maxCorrection);
    }

    public void moveAngle2(double distance, double angle, double turn) {
        drivetrain.moveAngle2(distance, angle, turn);
    }

    /**Shooter**/
    public void shootHighGoal(int distance) {
        shooter.shootHighGoal(distance);
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

    public void turnOdometry(double target, double power) {
        drivetrain.turnOdometry(target,power);
    }

    public void moveForwardOdometry(double distance, double power) {
        drivetrain.moveForwardOdometry(distance,power);
    }

    public void moveSideOdometry(double distance, double power) {
        //right is positive use distance to change direction
        drivetrain.moveSideOdometry(distance,power);
    }


    public void xyPath(double x, double y, double power) {
        drivetrain.xyPath(x,y,power);
    }

    public void StraightxyPath(double x, double y, double power) {
        drivetrain.StraightxyPath(x,y,power);
    }

    public void StraightPointPath(double x, double y, double power) {
        drivetrain.StraightPointPath(x,y,power);
    }

    public void DirectPointPath(double x, double y, double power) {
        drivetrain.DirectPointPath(x,y,power);
    }

    public void DirectxyPath(double x, double y, double power) {
        drivetrain.DirectxyPath(x,y,power);
    }
    public void moveAngleOdometry(double x, double y, double power){
        drivetrain.moveAngleOdometry(x,y,power);
    }


    /**
     * wobble goal methods
     */
    public void wobbleGoalStartingPosition(){
        if(isCorgi) {
            this.wobbleGoal.startingPosition();
        }
    }

    public void wobbleGoalGrabbingPosition(){
        if(isCorgi) {
            wobbleGoal.grabbingPosition();
        }
    }

    public void wobbleGoalLiftingPosition(){
        if(isCorgi) {
            wobbleGoal.liftingPosition();
        }
    }

    public void wobbleGoalDroppingPosition(){
        if(isCorgi) {
            wobbleGoal.droppingPosition();
        }
    }

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

    // ring depositor
    public void ringDepositorClockwise() {
        if(isCorgi) {
            ringDepositor.clockwise();
        }
    }

    public void ringDepositorCounterClockwise() {
        if(isCorgi) {
            ringDepositor.counterClockwise();
        }
    }

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

}