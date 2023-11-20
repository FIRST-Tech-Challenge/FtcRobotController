package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class RedudancyTestOne extends LinearOpMode {
    SampleMecanumDrive drive;

    
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        double y = 1;
        double x = 0;
        double lx = 0;

        boolean frwd = true;

        int threshold = 100;

        while (opModeIsActive() && threshold<100) {
          if (frwd) {
             y = 1;
          } else {
             y = 0;
          }
          drive.setMotorPowers(y + x + lx, y - x - lx, y - x + lx,y + x - lx);
          sleep(200);
        }
    }
}
