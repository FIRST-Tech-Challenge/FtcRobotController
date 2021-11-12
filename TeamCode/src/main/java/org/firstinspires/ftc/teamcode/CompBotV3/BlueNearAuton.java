package org.firstinspires.ftc.teamcode.CompBotV3;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotSimplified.CompBotHWSimplified;

import java.util.Arrays;

// Start blue storage side

@Autonomous

public class BlueNearAuton extends LinearOpMode {
    public static final double dPower = 0.25;
    ElapsedTime runtime = new ElapsedTime();
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry);
        telemetry.addLine("init finished");
        telemetry.update();
        r.lift.setPower(-1);
        boolean[] pos = {false,false,false};
        while(!isStarted()) {
            pos = r.p.getPositions();
        }
        runtime.reset();

        r.bucket.setPower(1);

        // line up with drop
        r.AEncDrive(0,29,0,dPower);
        r.AEncDrive(12,0,dPower,0);

        // lift and drop
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            nEncDrive(r.lift,1500,1);
            r.bucket.setPower(-1);
            nEncDrive(r.lift,-5000,-1);
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            nEncDrive(r.lift,3000,1);
            r.bucket.setPower(-1);
            nEncDrive(r.lift,-5000,-1);
        } else {// right
            nEncDrive(r.lift,4500,1);
            r.bucket.setPower(-1);
            nEncDrive(r.lift,-5000,-1);
        }
        r.bucket.setPower(1);

        // Go back
        r.AEncDrive(-16,0,-dPower,0);

        // Rotate to correct orientation
        r.gyroTurn(180,dPower);
        r.AEncDrive(4,0,0.15,0); // bang against wall

        // Strafe over to carousel
        r.AEncDrive(0,-36,0,-dPower);
        r.AEncDrive(0,-6,0,-0.15);

        // Turn on motor, wait, then turn off
        runMotorTime(r.spin,0.2,5000);

        // Strafe to warehouse
        r.AEncDrive(0,200,0,1);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}