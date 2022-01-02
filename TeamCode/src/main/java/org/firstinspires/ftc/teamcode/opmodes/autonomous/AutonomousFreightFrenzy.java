package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;


@Autonomous(name="Auto Freight Frenzy", group="FreightFrenzy")
public class AutonomousFreightFrenzy extends CommandOpMode {

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setAutoClear(false);

        final String[] selectedAlliance = new String[1];
        final String[] selectedPosition = new String[1];
        final String[] selectedPath = new String[1];
        boolean blueteam = false;

        GamepadEx settingsOp = new GamepadEx(gamepad1);

        schedule(new InstantCommand(() -> {
            telemetry.clearAll();
            telemetry.addLine("What is your Alliance?");
            telemetry.addLine("Press (X) for BLUE, (B) for RED");
            telemetry.addLine("-------------------------------");
            telemetry.addLine("What is your Position?");
            telemetry.addLine("Press (Y) for Position 1, (A) for Position 2");
            telemetry.update();
        }));



        //X (Blue) button
        Button blueAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.X);
        //Y (Red) button
        Button redAlliance = new GamepadButton(settingsOp, GamepadKeys.Button.B);

        //Y (Yellow) button
        Button position1 = new GamepadButton(settingsOp, GamepadKeys.Button.Y);
        //A (Red) button
        Button position2 = new GamepadButton(settingsOp, GamepadKeys.Button.A);

        Button path1Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_UP);
        Button path2Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_RIGHT);
        Button path3Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_DOWN);
        Button path4Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_LEFT);


        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin");

        //blueAlliance.whenReleased().and(position1.whenActive())

        blueAlliance.whenReleased(()->{
            selectedAlliance[0] = "Blue";

            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine("Blue Alliance Selected");
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }));

            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.BLUE);
            createLEDs.createAuto();


        });


        redAlliance.whenPressed(()->{
            selectedAlliance[0] = "Red";
            schedule(new InstantCommand(() -> {
                telemetry.clearAll();
                telemetry.addLine("Red Alliance Selected");
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }));
            Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.RED);
            createLEDs.createAuto();
        });

        path1Selector.whenPressed(()->{
            selectedPath[0] = "Path 1";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                DuckSideBluePath1 duckSideBluePath1 = new DuckSideBluePath1(hardwareMap,dashboard,telemetry);
                duckSideBluePath1.createPath();
                duckSideBluePath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                P1RedPath1 duckSideRedPath1 = new P1RedPath1(hardwareMap,dashboard,telemetry);
                duckSideRedPath1.createPath();
                duckSideRedPath1.execute(this);
            }
        });

        path2Selector.whenPressed(()->{
            selectedPath[0] = "Path 2";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
                DuckSideBluePath2 duckSideBluePath2 = new DuckSideBluePath2(hardwareMap, telemetry);
                duckSideBluePath2.createPath();
                duckSideBluePath2.execute(this);

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                DuckSideRedPath2 duckSideRedPath2 = new DuckSideRedPath2(hardwareMap,dashboard,telemetry);
                duckSideRedPath2.createPath();
                duckSideRedPath2.execute(this);
            }
        });

        path3Selector.whenPressed(()->{
            selectedPath[0] = "Path 3";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){

                DuckSideBluePath3 duckSideBluePath3 = new DuckSideBluePath3(hardwareMap, telemetry);
                duckSideBluePath3.createPath();
                duckSideBluePath3.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){
                DuckSideRedPath3 duckSideRedPath3 = new DuckSideRedPath3(hardwareMap,dashboard,telemetry);
                duckSideRedPath3.createPath();
                duckSideRedPath3.execute(this);
            }
        });

        path4Selector.whenPressed(()->{
            selectedPath[0] = "Path 4";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
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