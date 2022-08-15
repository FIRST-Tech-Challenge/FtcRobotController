package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "FWD")
public class FWD extends LinearOpMode
{
    private DcMotorEx wobbleArmGoal;
    private Servo wobbleGoalGrabbyer;
    private DcMotor motorLeftBack;
    private DcMotorEx discIntake;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0,0,0);
    public PIDCoefficients pidGains = new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // need to specify public static bc otherwise it no show up in the dashboard
    public static double ysensitivity = 0.3;
    public static double xsensitivity = 0.4;
    boolean isTurning = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        discIntake = (DcMotorEx)hardwareMap.dcMotor.get("discIntake");
        discIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArmGoal = (DcMotorEx)hardwareMap.dcMotor.get("wobbleArmGoal");
        wobbleGoalGrabbyer = hardwareMap.servo.get("wobbleGoalGrab");
        wobbleArmGoal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbleArmGoal.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        wobbleGoalGrabbyer.setPosition(0);
        waitForStart();

        resetStartTime();
        while(opModeIsActive()&&getRuntime()<90)
        {
            if(gamepad1.a){ //lifting
                wobbleArmGoal.setTargetPosition(122);
                wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                wobbleArmGoal.setPower(0.1);
            }
            else if(gamepad1.b){ //start
                wobbleArmGoal.setTargetPosition(45);
                wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                wobbleArmGoal.setPower(0.1);
            }
            if(gamepad1.y){
                wobbleGoalGrabbyer.setPosition(0);
            }
            if(gamepad1.x){
                wobbleGoalGrabbyer.setPosition(1);
            }

//            if(abs(gamepad1.left_trigger - gamepad1.right_trigger)>0.2) {
////                wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                wobbleArmGoal.setVelocity(200 * (gamepad1.left_trigger - gamepad1.right_trigger));
//            }
//            else{
//                wobbleArmGoal.setVelocity(0);
//            }
//            discIntake.setPower(gamepad1.right_stick_x*xsensitivity);
            if(gamepad1.right_bumper){
                discIntake.setPower(0.5);
            }
            else if(gamepad1.left_bumper){
                discIntake.setPower(0);
            }

            motorRightBack.setPower(gamepad1.left_stick_y*0.3+gamepad1.right_stick_x*0.4);
            motorLeftBack.setPower(gamepad1.left_stick_y*0.3-gamepad1.right_stick_x*0.4);
            motorRightFront.setPower(gamepad1.left_stick_y*0.3+gamepad1.right_stick_x*0.4);
            motorLeftFront.setPower(gamepad1.left_stick_y*0.3-gamepad1.right_stick_x*0.4);
            // sussy telemetry
            telemetry.addData("Left Back Motor Pos", motorLeftBack.getCurrentPosition());
            telemetry.addData("Left Front Motor Pos", motorLeftFront.getCurrentPosition());
            telemetry.addData("Right Back Motor Pos", motorRightBack.getCurrentPosition());
            telemetry.addData("Right Front Motor Pos", motorRightFront.getCurrentPosition());
            telemetry.addData("wobble goal arm pos", wobbleArmGoal.getCurrentPosition());
            // restart robit to make all values go back to default(0)
            telemetry.update();
            sleep(40);

        }
    }
    double integral = 0;
    double lastError = 0;
    public void setTargetPosAsync(int targetPos, double targetVelocity){

        PIDTimer.reset();

        double currentVelocity = wobbleArmGoal.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error*PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d * derivative;

        wobbleArmGoal.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }
}