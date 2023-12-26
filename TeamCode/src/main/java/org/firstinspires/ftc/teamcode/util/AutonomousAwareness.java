package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

public class AutonomousAwareness {
    /** Width of the track */
    static final double TRACK_WIDTH = 13.7;
    public final static FTCDashboardPackets dbp = new FTCDashboardPackets("AutoAware");

    /** Amount of ticks per one inch */
    static double TICKS_TO_INCHES;

    static final double CENTER_WHEEL_OFFSET = 2.4;
    static final double WHEEL_DIAMETER = 4.0; // inches

    private DcMotor encoderLeft, encoderRight, encoderBack;
    public HolonomicOdometry holOdom;
    
    public OdometrySubsystem odometry;

    public MecanumDrive m_robotDrive;
    public Motor fL, fR, bL, bR;

    public Path m_path = new Path();
    public PurePursuitCommand ppCommand;

    private Boolean usePurePursuit = false;

    public enum StartingPosition {
        RED_LEFT, 
        RED_RIGHT, 
        BLUE_LEFT, 
        BLUE_RIGHT
    }

    public void initOdometry(DcMotor encodeLeft, DcMotor encodeRight, DcMotor encodeBack) { //MotorEx encodeLeft, MotorEx encodeRight, MotorEx encodeBack) {
        dbp.createNewTelePacket();

        encoderLeft = encodeLeft;
        encoderRight = encodeRight;
        encoderBack = encodeBack;

        // Reset motors
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the directions of the odometry
        encoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the mode of the encoders
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        dbp.createNewTelePacket();
        dbp.put("Waypoints", Arrays.toString(m_path.toArray()));
        dbp.send(true);
        /*
        addToPath(new StartWaypoint());
        addToPath(new GeneralWaypoint(200, 0, 0.8, 0.8, 30));
        addToPath(new EndWaypoint());
        initPath();
        followPath();
         */
    }

    /** 
     * Constructor for the AutonomousAwareness class
     * @param startingPosition the starting position of the robot
     * @param _usePurePursuit whether or not to use a pure pursuit command
     */
    public AutonomousAwareness(StartingPosition startingPosition, boolean _usePurePursuit,
                               Motor _fL, Motor _fR, Motor _bL, Motor _bR,
                               DcMotor encodeLeft, DcMotor encodeRight, DcMotor encodeBack) {
        // Init motors
        fL = _fL;
        fR = _fR;
        bL = _bL;
        bR = _bR;

        if (fL == null) {
            dbp.error("Front Left Wheel is null");
            dbp.send(false);
            return;
        }

        usePurePursuit = _usePurePursuit;

        dbp.debug(usePurePursuit ? "Using pure pursuit" : "Not using pure pursuit");

        if (usePurePursuit) {
            createNewPurePursuitCommand();
        }

        initOdometry(encodeLeft, encodeRight, encodeBack);
    }

    public void addToPath(Waypoint waypoint) {
        dbp.debug("Adding waypoint to path", true, false);
        m_path.add(waypoint);
        if (usePurePursuit) ppCommand.addWaypoint(waypoint);
    }

    public void createNewPurePursuitCommand() {
        ppCommand = new PurePursuitCommand(
                m_robotDrive, odometry
        );
    }

    public void initPath() {
        try {
            dbp.debug("Initializing Path...", true, true);
            m_path.init();
        } catch (Exception e) {
            dbp.error(e, true, true);
        }
    }

    public void followPath() {
        try {
            dbp.debug("Following the path...", true, true);
            m_path.followPath(m_robotDrive, holOdom);
        } catch (IllegalStateException e) {
            dbp.error(e, true, true);
            //m_path.reset();
        }
    }

    public void resetPath() {
        dbp.debug("Resetting the path...", true, true);
        m_path.reset();
    }
}
