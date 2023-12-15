package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTCDashboardPackets;

import java.util.Arrays;

public class AutonomousAwareness {
    /** Width of the track */
    static final double TRACK_WIDTH = 13.7;
    public final static FTCDashboardPackets dbp = new FTCDashboardPackets();

    /** Amount of ticks per one inch */
    static double TICKS_TO_INCHES;

    static final double CENTER_WHEEL_OFFSET = 2.4;
    static final double WHEEL_DIAMETER = 4.0; // inches

    private static DcMotor encoderLeft, encoderRight, encoderBack;
    static HolonomicOdometry holOdom;
    
    static OdometrySubsystem odometry;

    public static MecanumDrive m_robotDrive;
    public static Motor fL, fR, bL, bR;

    public static Waypoint[] currentTask;
    public static Path m_path = new Path();
    private static PurePursuitCommand ppCommand;

    public enum StartingPosition {
        RED_LEFT, 
        RED_RIGHT, 
        BLUE_LEFT, 
        BLUE_RIGHT
    }

    public void initOdometry(DcMotor encodeLeft, DcMotor encodeRight, DcMotor encodeBack) { //MotorEx encodeLeft, MotorEx encodeRight, MotorEx encodeBack) {
        encoderLeft = encodeLeft;
        encoderRight = encodeRight;
        encoderBack = encodeBack;

        //TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / encoderLeft.getCPR();
        TICKS_TO_INCHES = 15.3;

        /*
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderBack.setDistancePerPulse(TICKS_TO_INCHES);
         */

        holOdom = new HolonomicOdometry(
            () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
            () -> encoderRight.getCurrentPosition() * TICKS_TO_INCHES,
            () -> encoderBack.getCurrentPosition() * TICKS_TO_INCHES,
            TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);

        m_robotDrive = new MecanumDrive(false, fL, fR, bL, bR);

        resetPath();
        dbp.createNewTelePacket();
        dbp.put("Waypoints", Arrays.toString(m_path.toArray()));
        dbp.send(true);

        addToPath(new StartWaypoint());
        addToPath(new GeneralWaypoint(200, 0, 0.8, 0.8, 30));
        addToPath(new EndWaypoint());
        initPath();
        followPath();
    }

    /** 
     * Constructor for the AutonomousAwareness class
     * @param startingPosition the starting position of the robot
     * @param useDistanceSensor whether or not to use the distance sensor for more accurate positioning
     */
    public AutonomousAwareness(StartingPosition startingPosition, boolean useDistanceSensor,
                               Motor fL, Motor fR, Motor bL, Motor bR,
                               DcMotor encodeLeft, DcMotor encodeRight, DcMotor encodeBack) {
        // Init motors
        AutonomousAwareness.fL = fL;
        AutonomousAwareness.fR = fR;
        AutonomousAwareness.bL = bL;
        AutonomousAwareness.bR = bR;

        if (AutonomousAwareness.fL == null) {
            dbp.createNewTelePacket();
            dbp.put("Error", "Front Left Wheel is null");
            dbp.send(false);
            return;
        }

        initOdometry(encodeLeft, encodeRight, encodeBack);
    }

    @Deprecated
    public static void newPursuit(StartWaypoint start, GeneralWaypoint general, EndWaypoint end) {
        ppCommand = new PurePursuitCommand(m_robotDrive, odometry, start, general, end);
        ppCommand.schedule();
    }

    public static void addToPath(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public static boolean initPath() {
        try {
            m_path.init();
        } catch (Exception e) {
            return false;
        }
        return true;
    }

    public static void followPath() {
        try {
            m_path.followPath(m_robotDrive, holOdom);
        } catch (IllegalStateException e) {
            dbp.createNewTelePacket();
            dbp.put("Follow Path", "Error");
            dbp.send(false);
            m_path.reset();
            return;
        }
    }

    public static void resetPath() {
        m_path.reset();
    }
}
