package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.Intake;
import org.firstinspires.ftc.teamcode.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Components.Accesories.Sticks;
import org.firstinspires.ftc.teamcode.Components.Accesories.Transfer;
import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.ChassisFactory;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Components.ObjectDetection.TensorFlow;

public class Robot {

    private LinearOpMode op = null;
    public final static boolean isCorgi = true;

    // Hardware Objects
    private BasicChassis drivetrain = null;
    private Intake intake = null;
    private Transfer transfer = null;
    private WobbleGoal wobbleGoal = null;
    private Shooter shooter = null;
    private final VuforiaWebcam vuforiaWebcam = null;
    private TensorFlow tensorFlow = null;
    private Sticks sticks = null;

    public Robot(LinearOpMode opMode, BasicChassis.ChassisType chassisType, boolean objectDetectionNeeded, boolean vuforiaNAVIGATIONneeded ) {
        op = opMode;
        //This link has a easy to understand explanation. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
        drivetrain= ChassisFactory.getChassis(chassisType,op,vuforiaNAVIGATIONneeded,isCorgi);

        if(objectDetectionNeeded){
            tensorFlow = new TensorFlow(op);
        }
        if(vuforiaNAVIGATIONneeded){
            //vuforiaWebcam = new VuforiaWebcam(op);
//            vuforiaWebcam.init(op);
        }
//        if(objectDetectionNeeded && vuforiaNAVIGATIONneeded){
//            throw new RuntimeException("They both can't be true.");
//        }
            intake = new Intake(op);
            transfer = new Transfer(op);
            wobbleGoal = new WobbleGoal(op,vuforiaNAVIGATIONneeded);
            shooter = new Shooter(op);
            sticks = new Sticks(op);

//        if (objectDetectionNeeded) {
//            tensorFlow.runTensorFlowWaitForStart();
//        }
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
    public void goToPositionWithoutStop(float x, float y, float angle, float power){drivetrain.goToPositionWithoutStop(x,y,angle,power);}
    public void moveBackward(double distance, double power) {
        drivetrain.moveBackward(distance, power);
    }
    public void setPosition(float xPosition,float yPosition, float newAngle){drivetrain.setPosition(xPosition,yPosition,newAngle);}
    public double[] track(){return drivetrain.track();}//track[0] is y value, track[1] is x value, and track[2] is angle
    public void goToPosition(double yPosition,double xPosition, double newAngle, double power){drivetrain.goToPosition(yPosition,xPosition,newAngle,power);}
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

    public void navigate(){
        drivetrain.navigate();
    }

    /**Vuforia**/

    public void startVuforia () {
        vuforiaWebcam.start();
    }

    public double getVuforiaAngle() {
        if(isCorgi) {
            return VuforiaWebcam.getVuforiaAngle();
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
        if(isCorgi) {
            double vuforiaX = VuforiaWebcam.getVuforiaX();
            double vuforiaY = VuforiaWebcam.getVuforiaY();
            double vuforiaAngle = vuforiaWebcam.getVuforiaAngle2();
            //robot angle using vuforia
            double robotAngle = vuforiaAngle + 90;
            robotAngle = (robotAngle > 180 ? robotAngle - 360 : robotAngle);
        }
    }
    public void stopVuforiaTF() {
        if(isCorgi) {
            vuforiaWebcam.interrupt();
        }
    }

    /**TensorFlow/Ring Detection**/
    public void stopRingDetection () {
        tensorFlow.stopTensorFlow();
    }

    public int getRingsAndWaitForStart(){
        return tensorFlow.runTensorFlowWaitForStart();
    }
    /**wobble goal methods**/
    public WobbleGoal.Position moveWobbleGoalToPosition(WobbleGoal.Position p){
            wobbleGoal.goToPosition(p);
        return (p);
    }

    public void printCurrentWobbleGoalLocation(){
            wobbleGoal.printCurrentLocation();
    }

    public void stopWobbleGoal(){
            wobbleGoal.stop();
    }

    public void openWobbleGoalClaw(){
        wobbleGoal.openWobbleGoalClaw();
    }

    public void closeWobbleGoalClaw(){
        wobbleGoal.closeWobbleGoalClaw();
    }

    // intake
    public void startIntake(){
            intake.startIntake();
    }

    public void reverseIntake(){
       intake.reverseIntake();
    }

    public void stopIntake(){
            intake.stopIntake();
    }

    //transfer
    public void startTransfer(){
            transfer.startTransfer();
    }

    public void reverseTransfer(){
        transfer.reverseTransfer();
    }

    public void stopTransfer() {
            transfer.stopTransfer();
    }

    //sticks
    public void moveLeftStick(double distance) {
        sticks.moveLeftStick(distance);
    }

    public void moveRightStick(double distance) {
        sticks.moveRightStick(distance);
    }

    //shooter

    public void moveServo(boolean direction) {
            shooter.moveServo(direction);
    }

    public void shootHighGoal(int rings) {
        shooter.shootHighGoal(rings);
    }

    public void shootHighGoalTest(double speed, int distance, int rings) {
        shooter.shootHighGoalTest(speed, distance, rings);
    }

    public void shootMiddleGoal(int rings){
        shooter.shootMidGoal(rings);
    }

    public void shootLowGoal(int rings){
        shooter.shootLowGoal(rings);
    }

    public void shootGoalTeleop(int distance) {
            shooter.shootGoalTeleop(distance);
    }

    public void shootPowershot(int distance){
        shooter.shootPowershot(distance);
    }

    public void stopShooter(){
        shooter.stopShooter();
    }
    public void setVelocity(double velocity, int distance){shooter.setVelocity(velocity,distance);}

    public void shootThreePowerShot() {
        ElapsedTime runtime = new ElapsedTime();
        op.telemetry.addData("speed: ", shooter.getRPM());
        op.telemetry.update();
        drivetrain.turnInPlace(-1.6,1.0);
        shooter.setVelocity(1375, 1000);
        op.sleep(300);
        if (shooter.getRPM()*28/60 > 0) {
            op.sleep(100);
            op.telemetry.clear();
            op.telemetry.addData("status", shooter.getRPM());
            op.telemetry.update();
        }
        shooter.moveServo(false);
        shooter.moveServo(true);
        shooter.setVelocity(1385, 1000);
        drivetrain.turnInPlace(3.1,0.6);
        shooter.moveServo(false);
        shooter.moveServo(true);
        drivetrain.turnInPlace(-7.0,0.5);
        shooter.moveServo(false);
        shooter.moveServo(true);
        transfer.stopTransfer();
        if(op.getRuntime()>3){
            stopShooter();
        }
    }
    public void shootThreePowerShotBounceBack() {
        ElapsedTime runtime = new ElapsedTime();
        op.telemetry.addData("speed: ", shooter.getRPM());
        op.telemetry.update();
        drivetrain.turnInPlace(-1.6,1.0);
        shooter.setVelocity(1330, 1000);
        op.sleep(200);
        if (shooter.getRPM()*28/60 > 0) {
            op.sleep(100);
            op.telemetry.clear();
            op.telemetry.addData("status", shooter.getRPM());
            op.telemetry.update();
        }
        shooter.moveServo(false);
        shooter.moveServo(true);
        shooter.setVelocity(1330, 1000);
        drivetrain.turnInPlace(3.3,0.6);
        shooter.moveServo(false);
        shooter.moveServo(true);
        drivetrain.turnInPlace(-7.0,0.5);
        shooter.moveServo(false);
        shooter.moveServo(true);
        transfer.stopTransfer();
        if(op.getRuntime()>3){
            stopShooter();
        }
    }

    public void shootThreePowerShotTransfer() {
        ElapsedTime runtime = new ElapsedTime();
        op.telemetry.addData("speed: ", shooter.getRPM());
        op.telemetry.update();
        drivetrain.turnInPlace(-1.6,1.0);
        shooter.setVelocity(1385, 1000);
        op.sleep(500);
        if (shooter.getRPM()*28/60 > 0) {
            op.sleep(100);
            op.telemetry.clear();
            op.telemetry.addData("status", shooter.getRPM());
            op.telemetry.update();
        }
        intake.stopIntake();
        transfer.startTransfer();
        shooter.moveServo(false);
        shooter.moveServo(true);
        transfer.reverseTransfer();
        shooter.setVelocity(1385, 1000);
        drivetrain.turnInPlace(3.3,0.6);
        shooter.moveServo(false);
        shooter.moveServo(true);
        transfer.startTransfer();
        drivetrain.turnInPlace(-7.2,0.5);
        shooter.moveServo(false);
        shooter.moveServo(true);
        transfer.stopTransfer();
        if(op.getRuntime()>3){
            stopShooter();
        }
    }

    public void intakeAndShootHighGoal(){
        ElapsedTime runtime = new ElapsedTime();
        shooter.setVelocity(1675, 1000);
        intake.startIntake();
        transfer.startTransfer();
        drivetrain.moveAngle(-20,20, 1.0);
        drivetrain.moveAngle(0,15, 1.0);
        shooter.moveServo(false);
    }
    public void navigateTeleOp(){
        drivetrain.navigateTeleOp();
    }
    public boolean goToPositionTeleop(double yPosition, double xPosition, double newangle, double power){return drivetrain.goToPositionTeleop(yPosition, xPosition, newangle, power);}
}
