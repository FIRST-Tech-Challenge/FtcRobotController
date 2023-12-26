package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.AutonomousAwareness;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

@Autonomous(name = "AutoCommandMode")
public class AutoCommandMode extends CommandOpMode {
    private AutonomousAwareness AA;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("AutoCommandMode");

    @Override
    public void initialize() {
        dbp.createNewTelePacket();
        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> motors = RobotHardwareInitializer.
                initializeDriveMotors(hardwareMap, this);

        Motor lFD;
        Motor rFD;
        Motor lBD;
        Motor rBD;

        // Temporary Fix
        try {
            lFD = new Motor(hardwareMap, "fl_drv");
            rFD = new Motor(hardwareMap, "fr_drv");
            lBD = new Motor(hardwareMap, "bl_drv");
            rBD = new Motor(hardwareMap, "br_drv");
        } catch(Exception e) {
            dbp.error("Could not initialize drive motors: " + e.getMessage(), false);
            terminateOpModeNow();
            return;
        }

        assert motors != null;

        final RobotHardwareInitializer.DriveMotor ENCODER_LEFT = RobotHardwareInitializer.DriveMotor.ENCODER_LEFT;
        final RobotHardwareInitializer.DriveMotor ENCODER_RIGHT = RobotHardwareInitializer.DriveMotor.ENCODER_RIGHT;
        final RobotHardwareInitializer.DriveMotor ENCODER_BACK = RobotHardwareInitializer.DriveMotor.ENCODER_BACK;

        AA = new AutonomousAwareness(AutonomousAwareness.StartingPosition.RED_LEFT, true,
                lFD, rFD, lBD, rBD,
                motors.get(ENCODER_LEFT), motors.get(ENCODER_RIGHT), motors.get(ENCODER_BACK));

        dbp.debug("Following Path", true);
        AA.addToPath(new StartWaypoint());
        AA.addToPath(new GeneralWaypoint(200, 0, 0.8, 0.8, 30));
        AA.addToPath(new EndWaypoint());
        /*
        AA.initPath();
        AA.followPath();
         */

        PurePursuitCommand ppCommand = new PurePursuitCommand(
                AA.m_robotDrive, AA.odometry,
                new StartWaypoint(),
                new GeneralWaypoint(200, 0, 0.8, 0.8, 30),
                new EndWaypoint()
        );
        dbp.debug("Scheduling ppCommand", true);
        schedule(ppCommand);
    }
}
