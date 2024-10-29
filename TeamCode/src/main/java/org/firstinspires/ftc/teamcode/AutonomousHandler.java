package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Objects;

public class AutonomousHandler {
    private HashMap<Integer, SystemState> path;
    private ArmSubSystem armSubSys;
    private DriveBaseSubsystem driveSubSys;
    private int systemStateReference = 0;
    private Telemetry theTelemetry;
    private FtcDashboard dashboard;

    public AutonomousHandler(HashMap<Integer, SystemState> pathList, ArmSubSystem armSubSystem, DriveBaseSubsystem driveBaseSystem, int StartAtStep, Telemetry thisTelemetry) {
        systemStateReference = StartAtStep;
        theTelemetry = thisTelemetry;
        path = pathList;
        driveSubSys = driveBaseSystem;
        armSubSys = armSubSystem;
        dashboard = FtcDashboard.getInstance();

        driveSubSys.setGoal(Objects.requireNonNull(path.get(systemStateReference)).drivePose);
        armSubSys.setReferences(Objects.requireNonNull(path.get(systemStateReference)).armPosition);
        armSubSys.setManipulatorReference(Objects.requireNonNull(path.get(systemStateReference)).wristPosition, Objects.requireNonNull(path.get(systemStateReference)).clawPosition);
    }
    public void periodicFunction() {
        TelemetryPacket packet = new TelemetryPacket();

        if (driveSubSys.isAtReference(packet) && armSubSys.isAtReference(packet)) {
            if (Objects.requireNonNull(path.get(systemStateReference + 1)).wristPosition == null) {
                theTelemetry.addData("driveSystemStopped", "yes");
            } else {
                theTelemetry.addData("driveSystemStopped", "changing");
                systemStateReference = systemStateReference + 1;
                driveSubSys.setGoal(Objects.requireNonNull(path.get(systemStateReference)).drivePose);
                armSubSys.setReferences(Objects.requireNonNull(path.get(systemStateReference)).armPosition);
                armSubSys.setManipulatorReference(Objects.requireNonNull(path.get(systemStateReference)).wristPosition, Objects.requireNonNull(path.get(systemStateReference)).clawPosition);
            }
        } else {
            theTelemetry.addData("driveSystemStopped", "no");
        }
        armSubSys.periodicUpdate(packet); // Moves arm
        driveSubSys.periodicUpdate(packet); // Moves Drive Base
        theTelemetry.addData("Auto Stage", systemStateReference);
        theTelemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}
