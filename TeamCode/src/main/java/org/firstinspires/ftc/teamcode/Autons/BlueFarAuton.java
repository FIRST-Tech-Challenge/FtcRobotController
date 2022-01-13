package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Blue Carousel Side")
public class BlueFarAuton extends LinearOpMode {
    public static final double dPower = 0.3;
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
            pos = r.p.getPositions();
        }

        r.phoneCam.stopStreaming();
        r.bucket.setPower(1);

        runtime.reset();

        // Turn
        r.AEncDrive(6,0,dPower,0);
        r.gyroTurn(90,dPower);
//        r.AEncDrive(6,0,dPower,0);

        // Strafe over
        r.AEncDrive(0,-24,0,-dPower);
        r.AEncDrive(0,-10,0,-dPower,2000);

        // Spin
        runMotorTime(r.spin,-0.2,2000);

        // Move over
        r.AEncDrive(0,56,0,dPower);
        r.gyroTurn(180,dPower);
        r.AEncDrive(-6,0,0.15,0,1500);

        // lift and drop
//        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
//            r.AEncDrive(20,0,dPower,0);
//            nEncDrive(r.lift,1300,1);
//            runMotorTime(r.bucket,-1,1000);
//            sleep(2000);
//            r.bucket.setPower(1);
//            driveUntilMechStop(r.lift,-1, 1000);
//            r.AEncDrive(-14,0,-dPower,0);
//        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
//            r.AEncDrive(16,0,dPower,0);
//            nEncDrive(r.lift,3000,1);
//            runMotorTime(r.bucket,-1,1000);
//            sleep(2000);
//            r.bucket.setPower(1);
//            driveUntilMechStop(r.lift,-1, 1000);
//            r.AEncDrive(-10,0,-dPower,0);
//        } else {// right
//            r.AEncDrive(10,0,dPower,0);
//            driveUntilMechStop(r.lift,1, 1000);
//            runMotorTime(r.bucket,-1,1000);
//            sleep(2000);
//            r.bucket.setPower(1);
//            driveUntilMechStop(r.lift,-1, 1000);
//            r.AEncDrive(-4,0,-dPower,0);
//        }
//        telemetry.addLine("finished with lift");
//        telemetry.update();
//        r.bucket.setPower(0);

        // Drive to depot
        r.AEncDrive(12,56,dPower,dPower);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}