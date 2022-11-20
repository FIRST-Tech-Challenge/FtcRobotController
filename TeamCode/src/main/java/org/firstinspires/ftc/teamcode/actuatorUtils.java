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
    //test
    private static int maxEncode = 4200; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int minEncode = 125; //Minimum so string on arm lift doesn't break and position 0
    private static int pos1 = 1850; //Low pole height
    private static int pos2 = 3000; //Med pole height
    private static int cone1 = 800; //Height for top cone in stack
    private static int cone2 = 700; //Height for second cone in stack
    private static int cone3 = 600; //Height for second cone in stack
    private static double armPower = .7f; //Set power to .7 so arm does not go up too fast


    //Initialize actuators
    public static void initializeActuator(DcMotor arm, Servo gripper) {
        actuatorUtils.arm = arm;
        actuatorUtils.gripper = gripper;

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
        gripper.setPosition(.6); //Position that grabs cone tightly
        if (liftUp == true) {
            sleep(500);
            arm.setTargetPosition(150); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
        }
    }

    //Method to open gripper
    public static void gripperOpen(boolean liftDown) throws InterruptedException {
        if (liftDown == true) {
            sleep(1000);
            arm.setTargetPosition(minEncode); //Lowers arm to min pos.
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
        }
        gripper.setPosition(.2); //Position to open gripper
    }

    //Method to move arm to pole heights
    public static void armPole(int desiredHeight) throws InterruptedException {
        if (desiredHeight == 1)
        {
            arm.setTargetPosition(pos1);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            sleep(2500);
        }
        else if(desiredHeight == 2)
        {
            arm.setTargetPosition(pos2);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            sleep(2500);
        }
        else if(desiredHeight == 3)
        {
            arm.setTargetPosition(maxEncode);
            //Set arm to RUN_TO_POSITION so we can effectively use the setTargetPosition() method
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            sleep(2500);
        }
    }

    //Method to move arm to cone collection heights
    public static void coneCollect (int desiredCone) throws InterruptedException {
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


}
