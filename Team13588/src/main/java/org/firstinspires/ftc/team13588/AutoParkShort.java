package org.firstinspires.ftc.team13588;

/* This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * This code assumes that we do not have encoders on the wheels,
 *
 * Our desire path:
 *  - Drive forward for 3 seconds
 *  - Spin right for 1.3 seconds
 *  - Drive Backward for 1 second
 *
 * The code is written in a simple for. No optimizations, and easy to be streamlined.
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.transition.Fade.IN;
import static org.firstinspires.ftc.team13588.RobotHardware.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.team13588.RobotHardware.WRIST_FOLDED_OUT;

@Autonomous(name = "Robot: Auto Drive by Time Short", group = "Robot")

public class AutoParkShort extends LinearOpMode {

    // OpMode members declared
    /*private DcMotor  leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    */
    private ElapsedTime   runtime = new ElapsedTime();

    static final double  FORWARD_SPEED = 0.6;
    static final double  TURN_SPEED    = 0.5;

    RobotHardware  robot =  new RobotHardware(this);
    @Override
    public void runOpMode(){

       robot.init();

        // telemetry message sent to signify robot.
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START basically)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        //Step 1: Drive forward for 3 seconds
        robot.armPosition = robot.ARM_WINCH_ROBOT;
        robot.wrist.setPosition(WRIST_FOLDED_IN);
       /*robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
       while (opModeIsActive() && (runtime.seconds() < 1.0)) {
           telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
           telemetry.update();
       }*/

        //Step 2: Strafe right for a while
        robot.driveRobot(0, FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.45)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 3: Drive backwards for a seconds.
        robot.driveRobot(FORWARD_SPEED, 0,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // strafe once more
       /* robot.driveRobot(0, FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Step 4 : Stop
        robot.driveRobot(0, 0,0);

        telemetry.addData("Path" , "Complete");
        telemetry.update();
        sleep(1000);

    }
}
