package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class actuatorUtils {
    private static CRServo spinspinducky = null;
    private static DcMotor armboom = null;
    private static CRServo intake = null;
    private static DcMotor LF = null;
    private static DcMotor LB = null;
    private static DcMotor RF = null;
    private static DcMotor RB = null;
    public static float ARM_POWER = .2f;


    public static void initializeActuator(DcMotor armboom, CRServo spinspinducky, CRServo intake) {
        actuatorUtils.armboom = armboom;
        actuatorUtils.spinspinducky = spinspinducky;
        actuatorUtils.intake = intake;

    }
    public static void initializeActuatorMovement(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB) {
        actuatorUtils.LF = LF;
        actuatorUtils.RF = RF;
        actuatorUtils.LB = LB;
        actuatorUtils.RB = RB;
    }

    public static void resetEncoders() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        armboom.setPower(0);

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
        armboom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Only using the Back motor Encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void spinThatDucky (boolean isRed) throws InterruptedException {
        resetEncoders();
        LF.setPower(.01);
        LB.setPower(.01);
        RF.setPower(.01);
        RB.setPower(.01);
        if (isRed) {
            spinspinducky.setPower(-1);
        }
        else {
            spinspinducky.setPower(1);
        }
        sleep(1500);
        resetEncoders();
        sleep(4500);
        spinspinducky.setPower(0);

    }

    public static void moveThatArm (float armRotationDistance){
        armboom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (armboom.getCurrentPosition() < armRotationDistance) {
            while (armboom.getCurrentPosition() < armRotationDistance) {
                armboom.setPower(ARM_POWER);
            }
        } else if (armboom.getCurrentPosition() > armRotationDistance) {
            while (armboom.getCurrentPosition() > armRotationDistance) {
                armboom.setPower(-ARM_POWER);
            }
        }
        armboom.setPower(0);
        armboom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void intakeMove(int i)
    {
        intake.setPower(i);
    }

}
