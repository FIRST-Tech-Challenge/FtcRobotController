package org.firstinspires.ftc.teamcode.teleOp;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Required import
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name = "201DistanceShoot - NEW")
public class DistanceShoot extends LinearOpMode {
    private DcMotor fLM, bLM, fRM, bRM;

    //MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++MAIN+++++++++
    public void runOpMode() {
        fLM = hardwareMap.get(DcMotor.class, "fLM");
        bLM = hardwareMap.get(DcMotor.class, "bLM");
        fRM = hardwareMap.get(DcMotor.class, "fRM");
        bRM = hardwareMap.get(DcMotor.class, "bRM");
        fLM.setDirection(DcMotor.Direction.REVERSE);
        bLM.setDirection(DcMotor.Direction.REVERSE);
        fRM.setDirection(DcMotor.Direction.FORWARD);
        bRM.setDirection(DcMotor.Direction.REVERSE);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        //backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


        //indexer.setPower(0.1);

        double max, forward, strafe, rotate, throttle, magnitude, angle, frontLeftPower, frontRightPower, backLeftPower, backRightPower;
        // Might slow down code a bit by declaring a new variable every time, so I moved it so it declares each variable once ^ at the start
        forward = (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
        strafe = (gamepad1.left_stick_x);
        rotate = (gamepad1.right_stick_x);
        // One way of making driving smoother
        magnitude = Math.hypot(forward, strafe);
        angle = Math.atan2(forward, strafe); //change to forward, strafe if doesn't work
        magnitude = Math.pow(magnitude, 2) * Math.signum(magnitude);
        forward = magnitude * Math.sin(angle);
        strafe = magnitude * Math.cos(angle);

        frontLeftPower = forward + strafe + rotate;
        frontRightPower = forward - strafe - rotate;
        backLeftPower = forward - strafe + rotate;
        backRightPower = forward + strafe - rotate;

        max = Math.max(1, Math.max(Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.abs(backLeftPower)), Math.abs(backRightPower)));
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

         /*LLResult llResult = limelights.getLatestResult();
         if (llResult != null && llResult.isValid()) {

             double tx = llResult.getTx();
             double turnPower = tx * tTP;

             if (Math.abs(tx) < sT) {
                 turnPower = 0;
             }

             turnPower = Math.max(-mP, Math.min(mP, turnPower));

             turnTable.setPower(turnPower);

             telemetry.addData("tag detected", true);
             telemetry.addData("tx (deg)", tx);
             telemetry.addData("turn power", turnPower);

         } else {
             // no tag -> stop turntable
             turnTable.setPower(0);
             telemetry.addData("Tag Detected", false);
         }

         telemetry.addData("Target RPM", tRPM);
         telemetry.addData("Actual RPM",
                 turretPower.getVelocity() * 60 / tPR);
         telemetry.update();

          */

        if (gamepad1.left_stick_y == 0 && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {
            fLM.setPower(0);
            fRM.setPower(0);
            bLM.setPower(0);
            bRM.setPower(0);
        }

        fLM.setPower(frontLeftPower);
        fRM.setPower(frontRightPower);
        bLM.setPower(backLeftPower);
        bRM.setPower(backRightPower);
//            frontLeftMotor.setPower(frontLeftPower * throttle);
//            frontRightMotor.setPower(frontRightPower * throttle);
//            backLeftMotor.setPower(backLeftPower * throttle);
//            backRightMotor.setPower(backRightPower * throttle);
    }
}
