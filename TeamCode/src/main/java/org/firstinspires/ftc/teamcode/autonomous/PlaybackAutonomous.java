package org.firstinspires.ftc.teamcode.autonomous;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

import java.io.DataInput;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

@Autonomous(name="Playback Auto", group="11588")
public class PlaybackAutonomous extends LinearOpMode {
    RI3WHardware robot = new RI3WHardware();
    ElapsedTime timer = new ElapsedTime();
    FileInputStream fileInputStream = new FileInputStream(Environment.getExternalStorageDirectory().getAbsolutePath() + "/test.log");
    DataInputStream readFile = new DataInputStream(fileInputStream);
    int lastKnownMilisecond = 0;
    double turningPower = 0;

    public PlaybackAutonomous() throws FileNotFoundException {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        timer.reset();
        while (true) {
            try {
                autoLoop();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void autoLoop() throws IOException {
        int currentMiliseconds = (int) Math.floor(timer.milliseconds());
        double ry = 0;
        double rx = 0;
        double ly = 0;
        double lx = 0;
        if (currentMiliseconds > lastKnownMilisecond) {
            lastKnownMilisecond = currentMiliseconds;
            ry = readFile.readFloat();
            rx = readFile.readFloat();
            ly = readFile.readFloat();
            lx = readFile.readFloat();
            turningPower = .75 * rx;
            double y = .75 * ly;
            double x = .75 * lx;
            //double y = impGamepad1.left_stick_y.getValue();
            //double x = impGamepad1.left_stick_x.getValue();
            double rightX = turningPower;

            robot.frontLeft.setPower(y + x + rightX);
            robot.frontRight.setPower(y - x - rightX);
            robot.backLeft.setPower(y - x + rightX);
            robot.backRight.setPower(y + x - rightX);
        }
    }

}
