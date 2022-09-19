package org.firstinspires.ftc.teamcode.League1.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LiftCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup.GrabAndLinkageUpGroup;
import org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup.ResetScoringGroup;

//TODO: FIX THIS


@TeleOp
public class FirstTeleOpRed extends CommandOpMode {

    private final double NORMAL_LINEAR_MODIFIER = 0.8;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.8;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    GamepadEx driver;
    GamepadEx beaconMechDriver;


    private Robot robot;
    public MecDrive drive;
    public ScoringSystem lift;
    public EndgameSystems endgameSystem;
    Constants constants;


    @Override
    public void initialize() {

        robot.start();

        //Gamepad extensions
        driver = new GamepadEx(gamepad1);
        beaconMechDriver = new GamepadEx(gamepad2);


        //TODO: Check all the commands and command groups
        //Close Grabber and Lift up linkage
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new GrabAndLinkageUpGroup(lift, constants, hardwareMap, robot));

        //Lift up slides to medium height
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new LiftCommand(lift, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.MEDIUM, 0.5));

        //Lift up slides to height height
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new LiftCommand(lift, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.HIGH, 0.5));

        //Slides back to zero, open grabber and linkage goes down
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ResetScoringGroup(lift, constants, hardwareMap, robot));


        //Initializing objects
        constants = new Constants();
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        lift = new ScoringSystem(hardwareMap, robot, constants,false);
        endgameSystem = new EndgameSystems(hardwareMap);


        //TODO: Fix this
        lift.setLinkagePosition(0.35);

    }



    @Override
    public void run() {
        super.run();



        //Drive code
        if (gamepad1.right_bumper) {
            drive.setPower(new Vector2D(driver.getLeftX() * SPRINT_LINEAR_MODIFIER, driver.getLeftY() * SPRINT_LINEAR_MODIFIER), driver.getRightX() * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            drive.setPower(new Vector2D(driver.getLeftX() * NORMAL_LINEAR_MODIFIER, driver.getLeftY() * NORMAL_LINEAR_MODIFIER), driver.getRightX() * NORMAL_ROTATIONAL_MODIFIER, false);
        }

        //TODO:Tune this value later
        if(robot.getRGBA(true).red > 500){
            schedule(new GrabAndLinkageUpGroup(lift, constants, hardwareMap, robot));

        }



        telemetry.update();
    }
}