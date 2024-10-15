package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class actuatorUtils {

    private static DcMotor lift = null; //declare arm

    private static Servo leftArm = null; //declare gripper
    private static Servo rightArm = null; //declare dump
    private static CRServo intake = null; //declare dump
    //test
    private static int maxEncode = 4200; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int minEncode = 300; //Minimum so string on arm lift doesn't break and position 0
    private static int lowEncode = 2000; //Minimum so string on arm lift doesn't break and position 0
    private static int highEncode = maxEncode; //Minimum so string on arm lift doesn't break and position 0
    private static double liftPower = .7f; //Set power to .7 so arm does not go up too fast

    enum LiftLevel
    {
        ZERO,
        LOW_BASKET,
        HIGH_BASKET

    }
    enum IntakeModes
    {
        OFF,
        IN,
        OUT

    }
    enum ArmModes
    {
        UP,
        DOWN

    }
    //Initialize actuators
    public static void initializeActuator(DcMotor lift, Servo leftArm, Servo rightArm, CRServo intake) {
        actuatorUtils.lift = lift;
        actuatorUtils.leftArm = leftArm;
        actuatorUtils.rightArm = rightArm;
        actuatorUtils.intake = intake;

    }






    //Method used to close gripper
    public static void setIntake(IntakeModes mode)  {
        if (mode == IntakeModes.IN) {
            intake.setPower(1.0);
        } else if (mode == IntakeModes.OUT) {
           intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    public static void setArm(ArmModes mode)   {
        if (mode == ArmModes.UP) {
            leftArm.setPosition(0.0);
            rightArm.setPosition(1.0);
        } else {
            leftArm.setPosition(1.0);
            rightArm.setPosition(0.0);
        }
    }
    public static void setLift(LiftLevel mode) {
        if (mode == LiftLevel.ZERO) {
            lift.setTargetPosition(minEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }  else if (mode == LiftLevel.LOW_BASKET) {
            lift.setTargetPosition(lowEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        } else {
            lift.setTargetPosition(highEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }
    }

    //Method to open gripper
    public static void gripperOpen(boolean liftDown) throws InterruptedException {
        gripperOpen(liftDown, true);
    }
    public static void gripperOpen() {
        gripper.setPosition(0.09);
    }
    public static void gripperClose() {
        gripper.setPosition(0.0);
    }
    public static void gripperOpen(boolean liftDown, boolean doSleep) throws InterruptedException {
        if (doSleep)
            sleep(500);
        gripper.setPosition(.2); //Position to open gripper
        if (liftDown == true) {
            if (doSleep)
                sleep(1000);
            arm.setTargetPosition(minEncode); //Lowers arm to min pos.
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
        }
    }

    public static void dumpClose()  {
        dump.setPosition(1); //Position that grabs cone tightly

    }

    //Method to open gripper

    public static void dumpOpen() {
        dump.setPosition(0.1); //Position to open gripper
    }
    //Method to move arm to pole heights
    public static void armPole(ArmLevel desiredHeight) throws InterruptedException {
        armPole(desiredHeight, true);
    }
    public static void armPole(int desiredHeight) throws InterruptedException {
        ArmLevel newHeight;
        if (desiredHeight==0)
            newHeight = ArmLevel.CONE1;
        else if (desiredHeight==4 || desiredHeight==1)
            newHeight = ArmLevel.LOW_POLE;
        else if (desiredHeight==2)
            newHeight = ArmLevel.MED_POLE;
        else
            newHeight = ArmLevel.HIGH_POLE;
        armPole(newHeight, true);
    }
    public static void armBoard() {
        arm.setTargetPosition(100);
        arm1.setTargetPosition(100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        arm1.setPower(0.5);

    }
    public static void noArmBoard() {
        arm.setTargetPosition(0);
        arm1.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        arm1.setPower(0.5);
    }
    public static void noElbowBoard() {
        elbow.setPosition(1);
    }
    public static void elbowBoard() {
        elbow.setPosition(0.6);
    }
    public static void armPole(ArmLevel desiredHeight, boolean doSleep) throws InterruptedException {
        if (desiredHeight == ArmLevel.LOW_POLE)
        {
            arm.setTargetPosition(lowPole);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(500);
        }
        else if(desiredHeight == ArmLevel.MED_POLE)
        {
            arm.setTargetPosition(medPole);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(2500);
        }
        else if(desiredHeight == ArmLevel.HIGH_POLE)
        {
            arm.setTargetPosition(highPole);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(2500);
        }
        else if(desiredHeight == ArmLevel.CONE5)
        {
            arm.setTargetPosition(cone5);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
        }
        else if (desiredHeight == ArmLevel.CONE4){
            arm.setTargetPosition(cone4);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
        }
        else if (desiredHeight == ArmLevel.CONE3){
            arm.setTargetPosition(cone3);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
        }
        else if (desiredHeight == ArmLevel.CONE2){
            arm.setTargetPosition(cone2);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
        }
        else if (desiredHeight == ArmLevel.CONE1){
            arm.setTargetPosition(cone1);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
        }
        else if (desiredHeight == ArmLevel.ZERO) {
            arm.setTargetPosition(0);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            if (doSleep)
                sleep(3000);
               }
    }

    //Method to move arm to cone collection heights
    /*public static void coneCollect (int desiredCone) throws InterruptedException {
        if (desiredCone == 1)
        {
            arm.setTargetPosition(cone1);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            sleep(1000);

        }
        else if(desiredCone == 2)
        {
            arm.setTargetPosition(cone2);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
        }
        else if(desiredCone == 3)
        {
            arm.setTargetPosition(cone3);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
        }
    }
     */


}
