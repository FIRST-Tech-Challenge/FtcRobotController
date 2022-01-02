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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BluePath1;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;

import java.util.concurrent.atomic.AtomicReference;


@Autonomous(name="Auto Freight Frenzy", group="FreightFrenzy")
public class AutonomousFreightFrenzy extends CommandOpMode {

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setAutoClear(false);

        final String[] selectedAlliance = new String[1];
        final String[] selectedPath = new String[1];
        boolean blueteam = false;

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

        Button path1Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_UP);
        Button path2Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_RIGHT);
        Button path3Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_DOWN);
        Button path4Selector = new GamepadButton(settingsOp, GamepadKeys.Button.DPAD_LEFT);


        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin");

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
                BluePath1 bluePath1 = new BluePath1(hardwareMap,telemetry);
                bluePath1.createPath();
                bluePath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){

            }
        });

        path2Selector.whenPressed(()->{
            selectedPath[0] = "Path 2";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){

            }
        });

        path3Selector.whenPressed(()->{
            selectedPath[0] = "Path 3";
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s",selectedPath[0],selectedAlliance[0]));
            telemetry.update();
            if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){

            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED){

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

        while (!isStarted()) {
            //run the scheduler before the start button is pressed
            CommandScheduler.getInstance().run();
        }

    }
}