package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ElapsedTime timer = new ElapsedTime();
    private double servoWaitStart = timer.milliseconds();
    private boolean waiting = false;
    private double stateStartTime = timer.milliseconds();

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
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("CapstanPosition", "INIT");
        packet.put("Capstan", "INIT");
        packet.put("x", 0);
        packet.put("y", 0);
        packet.put("Goaly", 0);
        packet.put("Goalx", 0);
        packet.put("GoalAngle", 0);
        packet.put("angle", 0);
        dashboard.sendTelemetryPacket(packet);
    }

    public void resetPID(float p, float i, float d) {
        driveSubSys.changePID(p, i, d);
    }

    public void periodicFunction() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Auto Stage", systemStateReference);
        if ((driveSubSys.isAtReference(packet, timer.milliseconds()) && armSubSys.isAtReference(packet)) || ((stateStartTime + (Objects.requireNonNull(path.get(systemStateReference)).ignorePosTime * 1000)) < timer.milliseconds())) {
            if (Objects.requireNonNull(path.get(systemStateReference + 1)).wristPosition == null) {
                theTelemetry.addData("driveSystemStopped", "yes");
                packet.put("driveSystemStopped", "yes");
            } else if ((!waiting) && ((Objects.requireNonNull(path.get(systemStateReference + 1)).wristPosition != Objects.requireNonNull(path.get(systemStateReference)).wristPosition) || (Objects.requireNonNull(path.get(systemStateReference + 1)).clawPosition != Objects.requireNonNull(path.get(systemStateReference)).clawPosition))) {
                servoWaitStart = timer.milliseconds();
                theTelemetry.addData("driveSystemStopped", "waitingForServo");
                packet.put("driveSystemStopped", "waitingForServo");
                waiting = true;
            } else if (timer.milliseconds() > (servoWaitStart + 1500)) {
                waiting = false;
                stateStartTime = timer.milliseconds();
                theTelemetry.addData("driveSystemStopped", "changing");
                packet.put("driveSystemStopped", "changing");
                systemStateReference = systemStateReference + 1;
                driveSubSys.setGoal(Objects.requireNonNull(path.get(systemStateReference)).drivePose);
                armSubSys.setReferences(Objects.requireNonNull(path.get(systemStateReference)).armPosition);
                armSubSys.setManipulatorReference(Objects.requireNonNull(path.get(systemStateReference)).wristPosition, Objects.requireNonNull(path.get(systemStateReference)).clawPosition);
            } else {
                packet.put("driveSystemStopped", "waitingForServo");
                theTelemetry.addData("driveSystemStopped", "waitingForServo");
            }
        } else {
            packet.put("driveSystemStopped", "no");
            theTelemetry.addData("driveSystemStopped", "no");
        }
        packet.put("timeToWait", servoWaitStart + 1500);
        packet.put("currentTime", timer.milliseconds());
        armSubSys.periodicUpdate(packet); // Moves arm
        driveSubSys.periodicUpdate(packet); // Moves Drive Base
        theTelemetry.addData("Auto Stage", systemStateReference);
        theTelemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}
