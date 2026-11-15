package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for Blue Team One.
 * Select this on the Driver Hub when our team is Blue Team One.
 * The starting pose must already be positioned and aimed for the preload shot.
 */
@Autonomous(name = "Blue Team One Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver")
public class BlueTeamOneAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.BLUE_1);
        waitForStart();
        auto.run();
    }
}
