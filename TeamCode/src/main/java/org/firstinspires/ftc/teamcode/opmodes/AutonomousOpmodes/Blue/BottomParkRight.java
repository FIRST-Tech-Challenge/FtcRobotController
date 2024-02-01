package org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmodes.Blue;

import static org.firstinspires.ftc.teamcode.autoutils.CompTrajectoryGenerator.trajectories.BLUE_BOTTOM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autoutils.CompTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

@Autonomous(name = "Blue Bottom - Park Right", group = "Blue", preselectTeleOp = "RealestDriverOpMode")
public class BottomParkRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        HashMap<RobotHardwareInitializer.Other, DynamicTypeValue> otherSystems =
                RobotHardwareInitializer.initializeAllOtherSystems(this);

        final CompTrajectoryGenerator CTG = new CompTrajectoryGenerator(drive,
                otherSystems, false);

        waitForStart();

        CTG.setStartingPosition(BLUE_BOTTOM);

        drive.followTrajectorySequence(CTG.BLUE_BOTTOM);
    }
}
