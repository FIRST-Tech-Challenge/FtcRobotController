package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="trackStyleTeleOpProgram")
public class trackStyleTeleOpProgram extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

//        robot.hwMap.get(DcMotor.class, "front_right_drive"); //expansion hub port 1 - direction: forward
//        robot.hwMap.get(DcMotor.class, "back_right_drive"); //expansion hub port 2 - direction: forward
//        robot.hwMap.get(DcMotor.class, "front_left_drive"); //expansion hub port 3 - direction: backward
//        robot.hwMap.get(DcMotor.class, "back_left_drive"); //expansion hub port 4 - direction: backward


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.setDrivetrainPower(-gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y*(2/3.0), -gamepad1.right_stick_y*(2/3.0));
            /*
            // Setup a variable for each drive side and mode (turn or forwards/backwards) to save power level for telemetry
            double driveLeftPower;
            double driveRightPower;
            double turnLeftPower;
            double turnRightPower;

            // Setup a variable for each lift mode (up or down) to save power level for telemetry
            double liftPower = 0.0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.

            //Since using tracks, provide power to motors for each side instead of each wheel.
            double drive = gamepad1.left_stick_y;
            double turn  = gamepad1.left_stick_x;


            //Providing power for motors to lift the lift
            boolean liftUp = gamepad1.dpad_up;
            boolean liftDown = gamepad1.dpad_down;

            if (liftUp)
            {
                liftPower = 1.0;
            }

            if (liftDown)
            {
                liftPower = -1.0;
            }

            double rightPower = drive - turn;
            double leftPower =  turn + drive;
            robot.setDrivetrainPowerTraditional(rightPower, leftPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "liftPower (%.2f)", liftPower);
            telemetry.update();

            */
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////


}
