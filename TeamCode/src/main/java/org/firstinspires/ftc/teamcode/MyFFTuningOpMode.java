package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.FeedforwardTuningOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class MyFFTuningOpMode extends FeedforwardTuningOpMode {
    public MyFFTuningOpMode() {
        super(72.0, MyMecanumDrive.MOTOR_CONFIG.getMaxRPM(), 4.0);
    }

    @Override
    protected Drive initDrive() {
        return new MyMecanumDrive(hardwareMap);
    }
}
