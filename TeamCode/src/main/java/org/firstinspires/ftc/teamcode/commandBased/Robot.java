package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.commandBased.commands.MoveArmIncrementally;
import org.firstinspires.ftc.teamcode.commandBased.commands.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.MoveElevator;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.AlignCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.LocalizerSubsystem;


import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Config
@TeleOp(name="Command Based", group="Linear Opmode")
public class Robot extends BlackOp {

    //declare subsystem variables
    public static DrivetrainSubsystem drivetrainSS;
    public static LocalizerSubsystem localizerSS;
    public static ElevatorSubsystem elevatorSS;
    public static ArmSubsystem armSS;

    public static double eleKg = 0.1;

    public static PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0);

    @Override
    public void go() {



        //cancel all previous commands
        CommandScheduler.getInstance().reset();

        //create subsystem objects
        drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        localizerSS = new LocalizerSubsystem(hardwareMap);
        elevatorSS = new ElevatorSubsystem(hardwareMap);
        armSS = new ArmSubsystem(hardwareMap);

        //create gamepads
        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        //create drivetrain mode commands
        FieldCentric fieldCentric = new FieldCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        RobotCentric robotCentric = new RobotCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        AlignCentric alignCentric = new AlignCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );

        //create elevator commands
        MoveElevator eleLow = new MoveElevator(elevatorSS, Constants.eleLow);
        MoveElevator eleHigh = new MoveElevator(elevatorSS, Constants.eleHigh);

        //create arm commands
        MoveArmIncrementally armForward = new MoveArmIncrementally(armSS, 10);
        MoveArmIncrementally armBackward = new MoveArmIncrementally(armSS, -10);
        MoveArmToAngle armIdle = new MoveArmToAngle(armSS, 0);

        //start robot in field-centric mode
        fieldCentric.schedule();

        waitForStart();

        Scheduler.launchOnStart(this, () -> {

            //drivetrain speed controls
            driver.left_bumper.onRise(() -> drivetrainSS.setSpeedMultipliers(0.5, 0.5, 0.5))
                              .onFall(() -> drivetrainSS.setSpeedMultipliers(1, 1, 1));

            //drivetrain mode controls
            driver.a.onRise(() -> {
                alignCentric.cancel();
                fieldCentric.cancel();
                robotCentric.schedule();
            });
            driver.b.onRise(() -> {
                alignCentric.cancel();
                robotCentric.cancel();
                fieldCentric.schedule();
            });
            driver.x.onRise(() -> {
                fieldCentric.cancel();
                robotCentric.cancel();
                alignCentric.schedule();
            });

            //elevator controls
            driver.dpad_up.onRise(eleHigh::schedule);
            driver.dpad_down.onRise(eleLow::schedule);

            //arm controls
            driver.dpad_right.onRise(armForward::schedule);
            driver.dpad_left.onRise(armBackward::schedule);
            driver.y.onRise(armIdle::schedule);

            mTelemetry().addData("target pos", elevatorSS.getEleTarget());
            mTelemetry().addData("ele pos", elevatorSS.getElePos());
            mTelemetry().addData("ele power", elevatorSS.getElePower());
            mTelemetry().update();

            //activate scheduler
            CommandScheduler.getInstance().run();
        });
    }
//    public void setEleKg(double Kg) {
//        Constants constants = new Constants();
//        constants.setEleKg(Kg);
//    }
}
