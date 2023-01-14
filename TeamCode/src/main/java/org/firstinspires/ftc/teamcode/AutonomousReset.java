/* FTC Team 7572 - Version 1.0 (11/26/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Autonomous reset program
 */

@Autonomous(name="Autonomous Robot-Reset", group="7592")
//@Disabled
public class AutonomousReset extends LinearOpMode {

    HardwareSlimbot robot = new HardwareSlimbot();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,true);

        // Bulk-query all odometry data (establish starting position)
        robot.readBulkData();

        // Check for test-mode updates (until operator presses PLAY).
        while (!isStarted()) {
            telemetry.addData("INFO", "Resets lift/turret/collector" );
            telemetry.addData("INFO", "to the correct starting" );
            telemetry.addData("INFO", "position for Autonomous" );
            telemetry.addData("State", "Ready");
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Perform the selected tests
        resetMechanismsForAutonomous();

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOpMode() */

    /*---------------------------------------------------------------------------------------------
     * getAngle queries the current gyro angle
     * @return  current gyro angle (-179.9 to +180.0)
     */
    public void performEveryLoop() {
        robot.readBulkData();
        robot.liftPosRun();
        robot.turretPosRun(false);
    }
    /*--------------------------------------------------------------------------------------------*/
    // Automatically move the robot mechanisms to the starting Autonomous configuration
    private void resetMechanismsForAutonomous() {
        // Extend collector while we're resetting the turret/lift
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB );
        // Ensure collector rotated to the upward position
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );
        // Initialize turret control (ensure we're centered!)
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        // Initialize lift control (lower to start position for autonomous)
        robot.liftPosInit( robot.LIFT_ANGLE_ASTART );
        // Wait until both motions are complete
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }
        // With arm in starting position, it's safe to store the collector
        robot.grabberSetTilt( robot.GRABBER_TILT_INIT );
        // Wait 1.5 seconds for grabber to actually tilt up before ending program
        sleep(1500);
    } // resetMechanismsForAutonomous

} /* AutonomousReset */
