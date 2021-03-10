package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

@TeleOp(name = "Shooter Velocity Tester", group = "Hardware Testers")
public class ShooterVelocityTester extends UpliftAuto {

    UpliftRobot robot;
    VelocityData velocityData;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        this.velocityData = robot.velocityData;
    }

    @Override
    public void initAction() {
        robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 25));
        robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 25));
    }

    @Override
    public void body() throws InterruptedException {
        waitForStart();

        Log.i("PID", robot.shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER) + "");

        robot.shooter1.setVelocity(robot.powerShotVelocity);
        robot.shooter2.setVelocity(robot.powerShotVelocity);


        double initialTime = System.currentTimeMillis();
        double timeElapsed = 0;
        while(timeElapsed < 10000 && opModeIsActive()) {
            Log.d("Shooter1 Velocity", robot.shooter1SmoothVel + "");
            Log.d("Shooter2 Velocity", robot.shooter2SmoothVel + "");
            Log.d("Time Elapsed", timeElapsed + "");
            Utils.sleep(50);
            timeElapsed = System.currentTimeMillis() - initialTime;
        }

        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);

    }

    @Override
    public void exit() {
        robot.stopThreads();
    }
}
