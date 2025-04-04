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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot: Auto Drive by Time", group = "Robot")
@Disabled

public class AutoParkLong extends LinearOpMode {

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

        //Step 1: Drive forward for 0.6 seconds
        robot.armPosition = robot.ARM_WINCH_ROBOT;
       robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
       while (opModeIsActive() && (runtime.seconds() < 0.58)) {
           telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
           telemetry.update();
       }

        //Step 2: strafe to the left for 1.2 seconds
        robot.driveRobot(0,  -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.23)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // move forward
        robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.45)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 3: Turn Left for a while
        robot.driveRobot(0, 0, -FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.45)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 4: strafe for 1 seconds.
        robot.driveRobot(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // move forward
        robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // strafe left
        robot.driveRobot(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // move forward
        robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         // Step 5: Drive backwards for 0.6 secs
        robot.driveRobot(FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //turn left
        robot.driveRobot(0, 0, -FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.17)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         // Step 6: move forward for 2.5 secs
        robot.driveRobot(-FORWARD_SPEED, 0,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.driveRobot(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 7: Drive forward for 1.0 secs
        robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.driveRobot(0, 0, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.40)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.driveRobot(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.45)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4 : Stop
        robot.driveRobot(0, 0,0);


        telemetry.addData("Path" , "Complete");
        telemetry.update();
        sleep(1000);

    }

}
