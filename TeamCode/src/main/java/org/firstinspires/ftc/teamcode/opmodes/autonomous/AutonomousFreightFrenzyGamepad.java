package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideBluePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideBluePath2;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideRedPath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.DuckSideRedPath2;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.WarehouseSideBluePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.WarehouseSideRedPath1;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;

@Autonomous(name="Auto Freight Frenzy Gamepad", group="FreightFrenzy")
public class AutonomousFreightFrenzyGamepad extends CommandOpMode {

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // telemetry.setAutoClear(false);

        final String[] selectedAlliance = new String[1];
        final String[] selectedSide = new String[1];
        final String[] selectedPath = new String[1];

        final Pose2d duckBlueStartPose = new Pose2d(-36, 60, Math.toRadians(270));
        final Pose2d duckRedStartPose = new Pose2d(-36, -60, Math.toRadians(90));
        final Pose2d warehouseBlueStartPose = new Pose2d(12, 60, Math.toRadians(270));
        final Pose2d warehouseRedStartPose = new Pose2d(12, -60, Math.toRadians(90));

        final Pose2d[] selectedStartPos = new Pose2d[1];

        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin");

        // Gamepad Selections during the Init loop
        schedule(new InstantCommand(() -> {

            boolean xPressed = false;
            boolean bPressed = false;
            boolean aPressed = false;
            boolean yPressed = false;

            while(!isStarted()) {

                if (gamepad1.x & !xPressed) {
                    selectedAlliance[0] = "Blue";
                    Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.BLUE);
                    createLEDs.createAuto();
                }
                xPressed = gamepad1.x;

                if (gamepad1.b & !bPressed) {
                    selectedAlliance[0] = "Red";
                    Alliance.getInstance().setAllicanceTeam(Alliance.AllianceTeam.RED);
                    createLEDs.createAuto();
                }
                bPressed = gamepad1.x;

                if (gamepad1.y & !yPressed) {
                    selectedSide[0] = "Duck";

                    if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                       selectedStartPos[0] = duckBlueStartPose;
                    } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                        selectedStartPos[0] = duckRedStartPose;
                    }
                }
                yPressed = gamepad1.y;

                if (gamepad1.a & !aPressed) {
                    selectedSide[0] = "Warehouse";

                    if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                        selectedStartPos[0] = warehouseBlueStartPose;
                    } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                        selectedStartPos[0] = warehouseRedStartPose;
                    }
                }
                aPressed = gamepad1.a;

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
                telemetry.addData("Side", selectedSide[0]);
                telemetry.addData("Path", selectedPath[0]);
                telemetry.addLine("___________________________________");
                telemetry.addLine("Press (X) for BLUE, (B) for RED");
                telemetry.addLine("Press (^) for Path 1");
                telemetry.addLine("Press (>) for Path 2");
                telemetry.addLine("Press (v) for Path 3");
                telemetry.addLine("Press (<) for Path 4");
                telemetry.update();
            }



        }));


        // Set the paths and run after start!
        schedule(new InstantCommand(() -> {

            telemetry.clearAll();
            telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s",selectedPath[0],selectedSide[0],selectedAlliance[0]));
            telemetry.update();

            // Path 1, start on duck end, spin the duck, deliver to the shipping hub, park in the storage container
            if (selectedPath[0] == "Path 1") {
                if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                    DuckSideBluePath1 bluePath1 = new DuckSideBluePath1(hardwareMap, selectedStartPos[0], telemetry);
                    bluePath1.createPath();
                    bluePath1.execute(this);
                } else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && selectedSide[0] == "Duck"){
                    DuckSideRedPath1 duckSideRedPath1 = new DuckSideRedPath1(hardwareMap,selectedStartPos[0],dashboard,telemetry);
                    duckSideRedPath1.createPath();
                    duckSideRedPath1.execute(this);
                }
                else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && selectedSide[0] == "Warehouse"){
                    WarehouseSideBluePath1 warehouseSideBluePath1 = new WarehouseSideBluePath1(hardwareMap,selectedStartPos[0],dashboard,telemetry);
                    warehouseSideBluePath1.createPath();
                    warehouseSideBluePath1.execute(this);
                }
                else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && selectedSide[0] == "Warehouse"){
                    WarehouseSideRedPath1 warehouseSideRedPath1 = new WarehouseSideRedPath1(hardwareMap,selectedStartPos[0],dashboard,telemetry);
                    warehouseSideRedPath1.createPath();
                    warehouseSideRedPath1.execute(this);
                }
            }

            // Path 2, start on duck end, spin the duck, deliver to the shipping hub, park in the storage container
            if (selectedPath[0] == "Path 2") {
                if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                    DuckSideBluePath2 duckSideBluePath2 = new DuckSideBluePath2(hardwareMap, selectedStartPos[0], telemetry);
                    duckSideBluePath2.createPath();
                    duckSideBluePath2.execute(this);
                } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                    DuckSideRedPath2 duckSideRedPath2 = new DuckSideRedPath2(hardwareMap,selectedStartPos[0], dashboard,telemetry);
                    duckSideRedPath2.createPath();
                    duckSideRedPath2.execute(this);
                }
            }

            if (selectedPath[0] == "Path 3") {
                if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {

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