package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.SummerMec;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFSlides;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "MecanumTeleOp(Weakened)")
public class MecanumTeleop extends LinearOpMode {

    LinearOpMode op = this;

    private final double MAX_INTAKE_EXTENSION_TICKS=545;
    private boolean isSequencing = false, isReversing = false, isRetracting = false,isExtending = false;
    private double[] time={100,100,0};

    RFSlides extendIntake;

    public void runOpMode() {

        ArrayList<Double> extensionCoefs = new ArrayList<>();
        extensionCoefs.add(4.0);
        extensionCoefs.add(400.0);


        DcMotorEx motorLeftFront;
        DcMotorEx motorRightFront;
        DcMotorEx motorLeftBack;
        DcMotorEx motorRightBack;
        DcMotorEx intakeMotor;
        Servo intakeFlip;

//        SummerMecRobot summermecrobot = new SummerMecRobot(this, true);
//
//        extendIntake = new RFSlides("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,
//                true, extensionCoefs, 545, 0);

        motorLeftFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
//        extendIntake = (DcMotorEx) op.hardwareMap.dcMotor.get("turret_Rotation");
//        extendIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);;
//        intakeMotor = (DcMotorEx) op.hardwareMap.dcMotor.get("intakeMotor");
//        extendIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeFlip = (Servo) op.hardwareMap.servo.get("intakeFlip");
//        OdometryTracker tracker = new OdometryTracker(false,false);
//        ColorDistanceRevV3 intakeSensor = new ColorDistanceRevV3();
//        tracker.setPosition(0,0,0);



        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
//            intakeFlip.setPosition(0.8);
        }
//        resetRuntime();
        while (!isStopRequested()&&getRuntime()<90) {
            float leftStickx = op.gamepad1.left_stick_x,leftSticky = op.gamepad1.left_stick_y,
                    rightStick = op.gamepad1.right_stick_x*0.5f;
            double leftStickr = sqrt(pow(leftSticky, 2) + pow(leftStickx, 2))*0.5,powera,powerb,
                    angle = atan2(leftSticky,leftStickx);

            powera = -sin(angle + PI/4);
            powerb = -sin(angle - PI/4);
//            double extendPower = op.gamepad1.right_trigger-op.gamepad1.left_trigger;
//            if(extendIntake.getCurrentPosition()<MAX_INTAKE_EXTENSION_TICKS&&extendPower>0||extendIntake.getCurrentPosition()>5&&extendPower<0) {
//                extendIntake.setPower((extendPower) * 1.0);
//                isExtending = true;
//                if(extendPower>0){
//                    intakeMotor.setPower(1.0);
//                }
//            }else{
//                isExtending = false;
//            }
            motorLeftFront.setPower(powerb * leftStickr + rightStick);
            motorRightBack.setPower(powerb * leftStickr - rightStick);
            motorRightFront.setPower(powera * leftStickr - rightStick);
            motorLeftBack.setPower(powera * leftStickr + rightStick);
//            op.telemetry.addData("extendTickles :p", extendIntake.getCurrentPosition());
//            tracker.track();
//            if(gamepad1.b){
//                intakeMotor.setPower(1.0);
//            }
//            else if(!isSequencing&&!isReversing&&!isExtending){
//                intakeMotor.setPower(0);
//            }
//            if(gamepad1.a&&getRuntime()-time[2]>0.2){
//                if(intakeFlip.getPosition()<0.3){
//                    intakeFlip.setPosition(0.8);
//                }else{
//                    intakeFlip.setPosition(0.1);
//                }
//                time[2]=getRuntime();
//            }
//            if(intakeSensor.getSensorDistance()<1||isSequencing){
//                if(time[0]>getRuntime()){
//                    time[0] = getRuntime();
//                    isRetracting = true;
//                    intakeFlip.setPosition(0.0);
//                    intakeMotor.setPower(1.0);
//                    retract();
//                    isSequencing = true;
//                }
//                if(isRetracting){
//                    retract();
//                }
//                if(time[0]+0.4<getRuntime()&&!isRetracting&&!isReversing){
//                    time[1] = getRuntime();
//                    intakeMotor.setPower(-1.0);
//                    telemetry.addData("retracting", isRetracting);
//                    telemetry.update();
//                    isReversing = true;
//                }
//                if(time[1]+0.5<getRuntime()){
//                    intakeMotor.setPower(0);
//                    intakeFlip.setPosition(0.8);
//                    time[0]=100;
//                    time[1]=100;
//                    isSequencing = false;
//                    isReversing=false;
//                }
//            }
        }
    }
//    public void retract(){
//        double power = 0.5, distance = extendIntake.getCurrentPosition();
//        if(distance<10){
//            extendIntake.setVelocity(0);
//            isRetracting = false;
//        }else{
//            extendIntake.setVelocity(-abs(5*distance+100));
//        }
//    }
}
