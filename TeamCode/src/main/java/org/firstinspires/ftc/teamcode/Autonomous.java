package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.CommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.IntakeServo;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Scheduler;
import org.firstinspires.ftc.teamcode.Commands.Turn;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {

    Scheduler scheduler = new Scheduler();
    DcMotor Arm1;
    PID PID = new PID(0.03, 0.0, 0.0);

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        waitForStart();

        scheduler.add(new CommandGroup(scheduler,
                new Drive(hardwareMap, 1, 1.4),
                new ParallelCommandGroup(scheduler, new Drive(hardwareMap,0.75, 1.4), new Intake(hardwareMap, 21, -0.5)),
                new ParallelCommandGroup(scheduler, new Turn(hardwareMap, 45), new Intake(hardwareMap, 1000, -0.5)),
                new ParallelCommandGroup(scheduler, new Drive(hardwareMap, 0.5, 0.2), new Intake(hardwareMap,2000, -0.5)),
                new Drive(hardwareMap, 0.5, -1),
/*                new Arm(hardwareMap, 200),*/
                new Drive(hardwareMap, -1, 2),
                new Turn(hardwareMap, 45),
                new Drive(hardwareMap, 1, 3)
        ));
        while (opModeIsActive())
        {
            /*
            PID.setSetPoint(-180);
            PID.updatePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            PID.setMaxInput(180);
            PID.setMinInput(-180);
            PID.setContinuous(true);
*/
            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("PID result", PID.getResult());
            telemetry.update();
            scheduler.update();
        }
    }
}