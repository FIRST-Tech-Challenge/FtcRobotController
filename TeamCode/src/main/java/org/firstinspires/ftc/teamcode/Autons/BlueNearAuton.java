package org.firstinspires.ftc.teamcode.Autons;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.driveUntilMechStop;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.nEncDrive;
import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3Attachments;

import java.util.Arrays;

// Start blue storage side

@Autonomous(name="Blue Warehouse Side")
public class BlueNearAuton extends LinearOpMode {
    public static final double dPower = 0.35;
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

        runtime.reset();

        r.bucket.setPower(1);

        // line up with drop
        r.AEncDrive(0,30,0,dPower);

        // lift and drop
        if (Arrays.equals(pos, new boolean[]{true, false, false})) {// left
            r.AEncDrive(20,0,dPower,0);
            nEncDrive(r.lift,1300,1);
            runMotorTime(r.bucket,-1,1000);
            sleep(2000);
            r.bucket.setPower(1);
            driveUntilMechStop(r.lift,-1, 1000);
            r.AEncDrive(-14,0,-dPower,0);
        } else if (Arrays.equals(pos, new boolean[]{false, true, false})) {// middle
            r.AEncDrive(16,0,dPower,0);
            nEncDrive(r.lift,3000,1);
            runMotorTime(r.bucket,-1,1000);
            sleep(2000);
            r.bucket.setPower(1);
            driveUntilMechStop(r.lift,-1, 1000);
            r.AEncDrive(-10,0,-dPower,0);
        } else {// right
            r.AEncDrive(10,0,dPower,0);
            driveUntilMechStop(r.lift,1, 1000);
            runMotorTime(r.bucket,-1,1000);
            sleep(2000);
            r.bucket.setPower(1);
            driveUntilMechStop(r.lift,-1, 1000);
            r.AEncDrive(-4,0,-dPower,0);
        }
        telemetry.addLine("finished with lift");
        telemetry.update();
        r.bucket.setPower(0);


        // Strafe to warehouse
        r.gyroTurn(90,0.2);
        telemetry.addLine("finished with turn");
        telemetry.update();
        r.AEncDrive(0,-10,0,-dPower, 3000);
        r.AEncDrive(0,-15,0,-0.15,3000);
        r.AEncDrive(100,0,1,0);

        r.stop();

        double rt = runtime.milliseconds();
        while(!isStopRequested()) {
            telemetry.addData("runtime",rt);
            telemetry.update();
        }
    }
}