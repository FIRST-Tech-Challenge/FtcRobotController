package org.firstinspires.ftc.teamcode;

import com.acmerobotics.splinelib.drive.Drive;
import com.acmerobotics.splinelib.drive.FeedforwardTuningOpMode;
import com.acmerobotics.splinelib.drive.MecanumDrive;
import com.acmerobotics.splinelib.trajectory.AssetsTrajectoryLoader;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import java.io.IOException;

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
