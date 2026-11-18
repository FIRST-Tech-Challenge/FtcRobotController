package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotVars;
import org.firstinspires.ftc.teamcode.mechaninisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechaninisms.Intake;
import org.firstinspires.ftc.teamcode.mechaninisms.Outtake;
import org.firstinspires.ftc.teamcode.mechaninisms.Stopper;
import org.firstinspires.ftc.teamcode.mechaninisms.Transfer;

@Autonomous(name = "FORWARD Auto", group = "Linear OpMode")
public class driveForwardAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        RobotVars robotVars = new RobotVars();
        Intake intake = new Intake(hardwareMap, robotVars);
        Transfer transfer = new Transfer(hardwareMap, robotVars);
        Outtake outtake = new Outtake(hardwareMap, robotVars);
        Stopper stopper = new Stopper(hardwareMap, robotVars, outtake);
        Drivetrain drive = new Drivetrain(hardwareMap, robotVars);


        while (!opModeIsActive()) { }

        //-----------------------------------------------------------------
        // Initialization ↑ and Start ↓
        //-----------------------------------------------------------------

        waitForStart();

        drive.runRobotCentricDriveCode(
                0,
                -0.9f,
                0
        );
        sleep(400);
        drive.runRobotCentricDriveCode(
                0,
                0,
                0
        );
        drive.setPowers(0, 0, 0, 0);
    }
}