package org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmodes.Red;

import static org.firstinspires.ftc.teamcode.autoutils.CompTrajectoryGenerator.trajectories.RED_TOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autoutils.CompTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

@Autonomous(name = "Red Top - Park Left", group = "Red", preselectTeleOp = "RealestDriverOpMode")
public class TopParkLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        HashMap<RobotHardwareInitializer.Other, DynamicTypeValue> otherSystems =
                RobotHardwareInitializer.initializeAllOtherSystems(this);

        final CompTrajectoryGenerator CTG = new CompTrajectoryGenerator(drive,
                otherSystems, true);

        waitForStart();

        CTG.setStartingPosition(RED_TOP);
                
        drive.followTrajectorySequence(CTG.RED_TOP);
        CTG.placePixelOnBoard();
                
        drive.setPoseEstimate(CTG.RED_TOP.end());
        drive.followTrajectorySequence(CTG.RED_TOP_PARK);
    }
}
