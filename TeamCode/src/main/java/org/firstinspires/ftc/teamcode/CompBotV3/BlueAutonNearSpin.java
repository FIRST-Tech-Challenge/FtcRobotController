package org.firstinspires.ftc.teamcode.CompBotV3;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

// Start blue storage side

@Autonomous
@Disabled

public class BlueAutonNearSpin extends LinearOpMode {
    public static final double dPower = 0.75;
    ElapsedTime runtime = new ElapsedTime();
    CompBotV3Attachments r = new CompBotV3Attachments();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true);
        boolean[] pos = {false,false,false};
        while(!isStarted()) {
             pos = r.p.getPositions();
        }
        runtime.reset();

        // line up with drop
        r.AEncDrive(24,0,dPower,0);
        r.AEncDrive(0,15,0,dPower);

        // lift and drop
        if (Arrays.equals(pos, new boolean[]{true, true, false})) {// left
            runMotorTime(r.lift,1,500);
            r.bucket.setPosition(0);
            runMotorTime(r.lift,-1,500);
        } else if (Arrays.equals(pos, new boolean[]{true, false, true})) {// middle
            runMotorTime(r.lift,1,1000);
            r.bucket.setPosition(0);
            runMotorTime(r.lift,-1,1000);
        } else {// right
            runMotorTime(r.lift,1,1500);
            r.bucket.setPosition(0);
            runMotorTime(r.lift,-1,1500);
        }

        // Go back
        r.AEncDrive(0,-16,0,-dPower);

        // Rotate to correct orientation
        r.gyroTurn(180,dPower);
        r.AEncDrive(0,4,0,0.15); // bang against wall

        // Strafe over to carousel
        r.AEncDrive(-36,0,-dPower,0);
        r.AEncDrive(6,0,-0.15,0);

        // Turn on motor, wait, then turn off
        runMotorTime(r.spin,0.2,5000);

        // Strafe to warehouse
        r.AEncDrive(200,0,1,0);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}