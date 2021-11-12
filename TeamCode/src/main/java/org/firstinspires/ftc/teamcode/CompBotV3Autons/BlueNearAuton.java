package org.firstinspires.ftc.teamcode.CompBotV3Autons;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotSimplified.CompBotHWSimplified;
import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous

public class BlueNearAuton extends LinearOpMode {
    public static final double dPower = 0.25;
    ElapsedTime runtime = new ElapsedTime();
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"blue");
        telemetry.addLine("init finished");
        telemetry.update();
        boolean[] pos = {false,false,false};
        ElapsedTime e = new ElapsedTime();
        while(!isStarted()) {
            if(e.milliseconds() < 5000) {
                //r.lift.setPower(-1);
            } else {
               // r.lift.setPower(0);
            }
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();

        runtime.reset();

        r.bucket.setPower(1);

        // line up with drop
        r.AEncDrive(0,33,0,dPower);
        r.AEncDrive(6,0,dPower,0);

        /* lift and drop
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
                */

        // Strafe to warehouse
        r.AEncDrive(-6,0,-dPower,0);
        r.AEncDrive(0,-200,0,-1);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}