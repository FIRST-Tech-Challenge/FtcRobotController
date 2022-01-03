package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideBluePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideBluePath2;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideBluePath3;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideRedPath2;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideRedPath3;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideRedPath1;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;


@Autonomous(name="Auto Freight Frenzy", group="FreightFrenzy")
public class AutonomousFreightFrenzy extends CommandOpMode {

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setAutoClear(false);

        final String[] selectedAlliance = new String[1];
        final String[] selectedSide = new String[1];
        final String[] selectedPath = new String[1];

        final Pose2d duckBlueStartPose = new Pose2d(-36, 60, Math.toRadians(270));
        final Pose2d duckRedStartPose = new Pose2d(-36, -60, Math.toRadians(90));
        final Pose2d warehouseBlueStartPose = null;
        final Pose2d warehouseRedStartPose = null;

        final Pose2d[] selectedStartPos = new Pose2d[1];

        GamepadEx settingsOp = new GamepadEx(gamepad1);

        schedule(new InstantCommand(() -> {
            telemetry.clearAll();
            telemetry.addLine("What is your Alliance?");
            telemetry.addLine("Press (X) for BLUE, (B) for RED");
            telemetry.update();
        }));



        //X (Blue) button
        Button blueAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.X);
        //Y (Red) button
        Button redAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.B);

        //Y (Yellow) button
        Button duckSide = new GamepadButton(settingsOp, GamepadKeys.Button.Y);
        //A (Green) button
        Button wharehouseSide = new GamepadButton(settingsOp, GamepadKeys.Button.A);

        Button path1Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_UP);
        Button path2Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_RIGHT);
        Button path3Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_DOWN);
        Button path4Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_LEFT);


        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin");


        blueAlliance.whenPressed(()->{
            selectedAlliance[0] = "Blue";

            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine("What is your Position?");
                telemetry.addLine("Press (Y) for Duck side, (A) for Wharehouse side");
                telemetry.update();
            }));

            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.BLUE);
            createLEDs.createAuto();

        });

        redAlliance.whenPressed(()->{
            selectedAlliance[0] = "Red";

            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine("What is your Position?");
                telemetry.addLine("Press (Y) for Duck side, (A) for Wharehouse side");
                telemetry.update();
            }));

            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.RED);
            createLEDs.createAuto();

        });

        duckSide.whenPressed(()->{

            selectedSide[0] = "Duck";
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                selectedStartPos[0] = duckBlueStartPose;

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                selectedStartPos[0] = duckRedStartPose;
            }

            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine(String.format("%s Alliance on Duck Side Selected",selectedAlliance[0]));
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }));
        });

        wharehouseSide.whenPressed(()->{
            selectedSide[0] = "Wharehouse";

            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                selectedStartPos[0] = warehouseBlueStartPose;

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                selectedStartPos[0] = warehouseRedStartPose;
            }

            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine(String.format("%s Alliance on Wharehouse side Selected",selectedAlliance[0]));
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }));
        });


        path1Selector.whenPressed(()->{
            selectedPath[0] = "Path 1";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s",selectedPath[0],selectedSide[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                DuckSideBluePath1 duckSideBluePath1 = new DuckSideBluePath1(hardwareMap, selectedStartPos[0], dashboard,telemetry);
                duckSideBluePath1.createPath();
                duckSideBluePath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                DuckSideRedPath1 duckSideRedPath1 = new DuckSideRedPath1(hardwareMap,selectedStartPos[0],dashboard,telemetry);
                duckSideRedPath1.createPath();
                duckSideRedPath1.execute(this);
            }
        });

        path2Selector.whenPressed(()->{
            selectedPath[0] = "Path 2";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s",selectedPath[0],selectedSide[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                DuckSideBluePath2 duckSideBluePath2 = new DuckSideBluePath2(hardwareMap, selectedStartPos[0], telemetry);
                duckSideBluePath2.createPath();
                duckSideBluePath2.execute(this);

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                DuckSideRedPath2 duckSideRedPath2 = new DuckSideRedPath2(hardwareMap,selectedStartPos[0], dashboard,telemetry);
                duckSideRedPath2.createPath();
                duckSideRedPath2.execute(this);
            }
        });

        path3Selector.whenPressed(()->{
            selectedPath[0] = "Path 3";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s",selectedPath[0],selectedSide[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){

                DuckSideBluePath3 duckSideBluePath3 = new DuckSideBluePath3(hardwareMap, selectedStartPos[0], telemetry);
                duckSideBluePath3.createPath();
                duckSideBluePath3.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                DuckSideRedPath3 duckSideRedPath3 = new DuckSideRedPath3(hardwareMap, selectedStartPos[0], dashboard,telemetry);
                duckSideRedPath3.createPath();
                duckSideRedPath3.execute(this);
            }
        });

        path4Selector.whenPressed(()->{
            selectedPath[0] = "Path 4";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s",selectedPath[0],selectedSide[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){

            }
        });


        while(!isStarted()) {

            CommandScheduler.getInstance().run();
        }

    }
}