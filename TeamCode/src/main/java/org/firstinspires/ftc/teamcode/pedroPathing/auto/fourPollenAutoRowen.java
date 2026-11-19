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

@Autonomous(name = "Four Pollen Auto Rowan", group = "Linear OpMode")
public class fourPollenAutoRowen extends LinearOpMode {
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

        intake.intakeMotor.setPower(-1);
        transfer.transferMotor.setPower(-1);
        stopper.runStopper_ToggleMode(false);
        outtake.runOuttake_ToggleMode(true, false);
        sleep(8000);
        stopper.stopperServo.setPosition(0.12);
        sleep(3000);
        drive.runRobotCentricDriveCode(
                0,
                0.9f,
                0
        );
        sleep(1000);
        drive.runRobotCentricDriveCode(
                0,
                0,
                0
        );
        drive.setPowers(0, 0, 0, 0);
    }
}