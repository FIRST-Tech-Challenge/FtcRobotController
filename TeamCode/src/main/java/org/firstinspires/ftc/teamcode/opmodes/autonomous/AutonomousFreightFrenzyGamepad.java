package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BluePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BluePath2;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="AutonomousPath1", group="FTCLib")
public class AutonomousFreightFrenzyGamepad extends CommandOpMode {

    @Override
    public void initialize() {

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // telemetry.setAutoClear(false);

        final String[] selectedAlliance = new String[1];
        final String[] selectedPath = new String[1];

        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin");

        // Gamepad Selections during the Init loop
        schedule(new InstantCommand(() -> {

            boolean xPressed = false;
            boolean bPressed = false;

            while(!isStarted()) {

                if (gamepad1.x & !xPressed) {
                    selectedAlliance[0] = "Blue";
                    Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.BLUE);
                }
                xPressed = gamepad1.x;

                if (gamepad1.b & !bPressed) {
                    selectedAlliance[0] = "Red";
                    Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.RED);
                }
                bPressed = gamepad1.x;

                if(gamepad1.dpad_up){
                    selectedPath[0] = "Path 1";
                }
                else if(gamepad1.dpad_right){
                    selectedPath[0] = "Path 2";
                }
                else if(gamepad1.dpad_down){
                    selectedPath[0] = "Path 3";
                }
                else if(gamepad1.dpad_left) {
                    selectedPath[0] = "Path 4";
                }

                telemetry.addData("Alliance", selectedAlliance[0]);
                telemetry.addData("Path", selectedPath[0]);
                telemetry.addLine("___________________________________");
                telemetry.addLine("Press (X) for BLUE, (B) for RED");
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }

            createLEDs.createAuto();

        }));


        // Set the paths and run after start!
        schedule(new InstantCommand(() -> {

            telemetry.clearAll();
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Path: %s", selectedPath[0], selectedAlliance[0]));
            telemetry.update();

            // Path 1, start on duck end, spin the duck, deliver to the shipping hub, park in the storage container
            if (selectedPath[0] == "Path 1") {
                if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                    BluePath1 bluePath1 = new BluePath1(hardwareMap, telemetry);
                    bluePath1.createPath();
                    bluePath1.execute(this);
                } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                    /*
                    RedPath1 redPath1 = new RedPath1(hardwareMap,telemetry);
                    redPath1.createPath();
                    redPath1.execute(this);
                    */
                }
            }

            // Path 2, start on duck end, spin the duck, deliver to the shipping hub, park in the storage container
            if (selectedPath[0] == "Path 2") {
                if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                    BluePath2 bluePath2 = new BluePath2(hardwareMap, telemetry);
                    bluePath2.createPath();
                    bluePath2.execute(this);
                } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                    /*
                    RedPath1 redPath1 = new RedPath1(hardwareMap,telemetry);
                    redPath1.createPath();
                    redPath1.execute(this);
                    */
                }
            }

        }));

    }
}