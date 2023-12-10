package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class actuatorUtils {
    private static DcMotor LF = null; //declare left front motor
    private static DcMotor LB = null; //declare left back motor
    private static DcMotor RF = null; //declare right front motor
    private static DcMotor RB = null; //declare right back motor
    private static DcMotor arm = null; //declare arm
    private static Servo gripper = null; //declare gripper
    private static Servo dump = null; //declare dump
    private static Servo elbow = null; //declare dump
    //test
    private static int maxEncode = 4200; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int minEncode = 300; //Minimum so string on arm lift doesn't break and position 0
    private static int highPole = maxEncode; //high pole height
    private static int medPole = 3100; //Med pole height
    private static int lowPole = 1900; //Low pole height
    private static int cone5 = 685; //Height for top cone in stack
    private static int cone4 = 475; //Height for second cone in stack
    private static int cone3 = 203; //Height for third cone in stack
    private static int cone2 = 36; //height for second cone in stack
    private static int cone1 = 9; //height for first cone in stack
    private static double armPower = .7f; //Set power to .7 so arm does not go up too fast

    enum ArmLevel
    {
        ZERO,
        CONE1,
        CONE2,
        CONE3,
        CONE4,
        CONE5,
        LOW_POLE,
        MED_POLE,
        HIGH_POLE
    }

    //Initialize actuators
    public static void initializeActuator(DcMotor arm, Servo gripper, Servo dump, Servo elbow) {
        actuatorUtils.arm = arm;
        actuatorUtils.gripper = gripper;
        actuatorUtils.dump = dump;
        actuatorUtils.elbow = elbow;

    }
    public static void initializeActuator(DcMotor arm, Servo gripper) {
        actuatorUtils.arm = arm;
        actuatorUtils.gripper = gripper;
        actuatorUtils.dump = null;
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //Initialize motors for movement
    public static void initializeActuatorMovement(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB) {
        actuatorUtils.LF = LF;
        actuatorUtils.RF = RF;
        actuatorUtils.LB = LB;
        actuatorUtils.RB = RB;
    }

    //Stop movement of wheels
    public static void resetEncoders() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        arm.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Just a little time to make sure encoders have reset
        //sleep(200);

        // Not technically encoder but...
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Only using the Back motor Encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Method used to close gripper
    public static void gripperClose(boolean liftUp) throws InterruptedException {
        gripperClose(liftUp, true);
    }
    public static void gripperClose(boolean liftUp, boolean doSleep) throws InterruptedException {
        gripper.setPosition(.6); //Position that grabs cone tightly
        if (liftUp == true) {
            if (doSleep)
                sleep(250);
            arm.setTargetPosition(minEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
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
        arm.setTargetPosition(-100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
    public static void noArmBoard() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
    public static void noElbowBoard() {
        elbow.setPosition(1);
    }
    public static void elbowBoard() {
        elbow.setPosition(0.579);
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
