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

        AA = new AutonomousAwareness(AutonomousAwareness.StartingPosition.RED_LEFT, true,
                lFD, rFD, lBD, rBD,
                motors.get("encoderLeft"), motors.get("encoderRight"), motors.get("encoderBack"));

        dbp.debug("Following Path", true);
        AA.addToPath(new StartWaypoint());
        AA.addToPath(new GeneralWaypoint(200, 0, 0.8, 0.8, 30));
        AA.addToPath(new EndWaypoint());
        /*
        AA.initPath();
        AA.followPath();
         */
        AA.ppCommand.schedule();
    }
}
