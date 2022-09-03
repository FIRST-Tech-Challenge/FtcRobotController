// CONTROLS - UPDATED 8/19/22 6:02 PM
//
package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "FWD")
public class FWD extends LinearOpMode {
    private DcMotorEx wobbleArmGoal;
    private Servo wobbleGoalGrabbyer;
    private CRServo tapeExtend;
    private Servo tapeHeight;
    private Servo tapeMove;
    private DcMotor motorLeftBack;
    private DcMotorEx discIntake;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 1680 / 360;

    // need to specify public static bc otherwise it no show up in the dashboard
    public static double ysensitivity = 0.3;
    public static double xsensitivity = 0.4;
    boolean isTurning = true;

    public static double speed = 1200; //arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned probably through FTC dashboard
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        discIntake = (DcMotorEx) hardwareMap.dcMotor.get("discIntake");
        discIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArmGoal = (DcMotorEx) hardwareMap.dcMotor.get("wobbleArmGoal");
        wobbleGoalGrabbyer = hardwareMap.servo.get("wobbleGoalGrab");
        tapeExtend = hardwareMap.crservo.get("tapeExtend");
        tapeHeight = hardwareMap.servo.get("tapeHeight");
        tapeMove = hardwareMap.servo.get("tapeMove");
        controller = new PIDController(p,i,d);

        wobbleArmGoal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbleArmGoal.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        wobbleGoalGrabbyer.setPosition(0);
        f = 0; p = 0; d = 0; i = 0;
        waitForStart();
        int onetwo = 0;
        int twoone = 0;

        resetStartTime();
        while (opModeIsActive() && getRuntime() < 90) {
            //wobbleArmGoal.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p,i,d,f));

            if(gamepad1.a){ //lifted pos
                target = 450;
            }
            else if(gamepad1.b){ //pickup pos
                target = 45;
            }

            controller.setPID(p,i,d);
            int armPos = wobbleArmGoal.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            double diff = armPos - target;
            wobbleArmGoal.setPower(power);
//            if(diff<0 && abs(diff)>20 ) {
//                wobbleArmGoal.setVelocity(50);
//            }
//            else if(diff>0 && abs(diff)<20){
//                wobbleArmGoal.setVelocity(-50);
//            }
//            else{
//                wobbleArmGoal.setVelocity(0);
//            }
            // NO WEIGHT PIDF VALUES:
            // d = 0, f = 0.006, i = 0, p = 0.002
            // WEIGHT PIDF VALUES:
            // PIDF REFERENCE: f = gravity resist, d = dampening of oscillations, i = idk, p = power-sorta thing
            //460 and 45
            if (gamepad1.y) {
                wobbleGoalGrabbyer.setPosition(0);
                f = 0;
                p = 0.00025;
            }
            if (gamepad1.x) {
                wobbleGoalGrabbyer.setPosition(1);
                p = 0.0004;
            }

//            if(abs(gamepad1.left_trigger - gamepad1.right_trigger)>0.2) {
////                wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                wobbleArmGoal.setVelocity(200 * (gamepad1.left_trigger - gamepad1.right_trigger));
//            }
//            else{
//                wobbleArmGoal.setVelocity(0);
//            }
//            discIntake.setPower(gamepad1.right_stick_x*xsensitivity);
            if (gamepad1.right_bumper) {
                discIntake.setPower(0.5);
            }
            else if (gamepad1.left_bumper) {
                discIntake.setPower(0);
            }
            tapeExtend.setPower(gamepad1.right_stick_y);
            if(gamepad1.dpad_left){
                onetwo += 1;
                tapeMove.setPosition(onetwo%2);
            }
            if(gamepad1.dpad_right){
                twoone += 1;
                tapeHeight.setPosition(twoone%2);
            }

            motorRightBack.setPower(gamepad1.left_stick_y * 0.3 );
            motorLeftBack.setPower(gamepad1.left_stick_y * 0.3 );
            motorRightFront.setPower(gamepad1.left_stick_y * 0.3 );
            motorLeftFront.setPower(gamepad1.left_stick_y * 0.3 );
            // sussy telemetry
            telemetry.addData("Left Back Motor Pos", motorLeftBack.getCurrentPosition());
            telemetry.addData("Left Front Motor Pos", motorLeftFront.getCurrentPosition());
            telemetry.addData("Right Back Motor Pos", motorRightBack.getCurrentPosition());
            telemetry.addData("Right Front Motor Pos", motorRightFront.getCurrentPosition());
            telemetry.addData("wobble goal arm pos", wobbleArmGoal.getCurrentPosition());
            telemetry.addData("velo", wobbleArmGoal.getVelocity());
            telemetry.addData("pos", wobbleArmGoal.getCurrentPosition());
            telemetry.addData("target", target);
            // restart robit to make all values go back to default(0)
            telemetry.update();
            sleep(40);

        }
    }

    double lastError = 0;
    double integral = 0;
    //initializing our variables
/*
    public void PID(double targetVelocity){
        PIDTimer.reset(); //resets the timer

        double currentVelocity = wobbleArmGoal.getVelocity();
        double error = targetVelocity - currentVelocity; //pretty self explanatory--just finds the error

        double deltaError = error - lastError; //finds how the error changes from the previous cycle
        double derivative = deltaError / PIDTimer.time(); //deltaError/time gives the rate of change (sensitivity of the system)

        integral += error * PIDTimer.time();
        //continuously sums error accumulation to prevent steady-state error (friction, not enough p-gain to cause change)

        pidGains.p = error * pidCoeffs.p;
        //acts directly on the error; p-coefficient identifies how much to act upon it
        // p-coefficient (very low = not much effect; very high = lots of overshoot/oscillations)
        pidGains.i = integral * pidCoeffs.i;
        //multiplies integrated error by i-coefficient constant
        // i-coefficient (very high = fast reaction to steady-state error but lots of overshoot; very low = slow reaction to steady-state error)
        // for velocity, because friction isn't a big issue, only reason why you would need i would be for insufficient correction from p-gain
        pidGains.d = derivative * pidCoeffs.d;
        //multiplies derivative by d-coefficient
        // d-coefficient (very high = increased volatility; very low = too little effect on dampening system)

        wobbleArmGoal.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
        //adds up the P I D gains with the targetVelocity bias

        lastError = error;
        //makes our current error as our new last error for the next cycle
    }
}
 */
}