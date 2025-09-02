package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Manual New Version", group="")
public class ManualOOP extends LinearOpMode{
    private HornetRobo hornetRobo = new HornetRobo() ;

    private LogManager logManager = new LogManager(this.telemetry, "Manual New Version");
    private ManualManager manualManager = new ManualManager(this, hornetRobo);

    private boolean isSlow = false;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        manualManager.Init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            logManager.WriteLog("Status", "Ready to run...");
            

            manageDriveMotors();
            manageGrabber();
            manageArm();

            manualManager.SetViperSlideMotorTargetPosition();
            manualManager.SetMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.left_stick_y > 0){
                manualManager.SetViperSlideDirectionForward();
                logManager.WriteLog("Status", "Left Joystick Moved up");
                manualManager.SetViperSlideMotorPower(0.5);
            }
            if(gamepad1.left_stick_y < 0){
                manualManager.SetViperSlideDirectionReverse();
                logManager.WriteLog("Status", "Left Joystick Moved down");
                manualManager.SetViperSlideMotorPower(0.5);
            }

            if(gamepad1.left_stick_y==0){
                manualManager.SetViperSlideMotorPower(0);
            }
            //below is the arm
           /* if(gamepad1.right_stick_y > 0){
                robot.setArmDirectionForward();
                logManager.WriteLog("Status", "Right Joystick Moved up");
                robot.setArmPower();
            }
            if(gamepad1.right_stick_y < 0){
                robot.setArmDirectionReverse();
                logManager.WriteLog("Status", "Right Joystick Moved down");
                robot.setArmPower();
            }
            if(gamepad1.right_stick_y==0){
                robot.setArmMotorPowerZero();
            }
*/
            if(gamepad2.right_trigger > 0){
                //robot.goDiagonal(1);
                manualManager.GoStrafe(gamepad2.right_trigger * -1);
            }
            else if (gamepad2.left_trigger > 0){
                manualManager.GoStrafe(gamepad2.left_trigger * 1);

            }

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
            
        }
    }

    private void manageDriveMotors(){
        double drive        = 0;
        double turn         = 0;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad2.right_stick_x;
        turn  = gamepad2.right_stick_y;

        //if button on gamepad2 is pressed
        //then drive and turn should be half
        if (gamepad2.a){
            isSlow = true;
        }
        if (gamepad2.b){
            isSlow = false;
        }

        if (isSlow){
            drive = drive / 2;
            turn = turn / 2;
        }

        // Combine drive and turn for blended motion. Use RobotHardware class
        manualManager.DriveRobot(drive, turn);

        logManager.WriteLog("Drivexxxxx", "Left Stick");
        logManager.WriteLog("Turn", "Right Stick");
        logManager.WriteLog("-", "-------");

        logManager.WriteLog("Drive Power", Double.toString(drive));
        logManager.WriteLog("Turn Power",  Double.toString(turn));
        
    }

    private void manageGrabber(){

        logManager.WriteLog("Grabber Key", Boolean.toString(gamepad1.a || gamepad1.b));
        if (gamepad1.a ) {
            manualManager.MoveGrabber(true);
        }
        else if (gamepad1.b ) {
            manualManager.MoveGrabber(false);
        }
    }

    private void manageArm(){

        if (gamepad1.right_stick_y > 0) {
            logManager.WriteLog("ManageArm: Y Stick", Float.toString(gamepad1.right_stick_y));
            manualManager.MoveArm(true);
        }
        else if (gamepad1.right_stick_y < 0) {
            logManager.WriteLog("ManageArm: Y Stick", Float.toString(gamepad1.right_stick_y));
            manualManager.MoveArm(false);
        }

        if (gamepad1.x){
            logManager.WriteLog("Manage Arm: X - Arm Home Postion : ", Boolean.toString(gamepad1.x));

            manualManager.MoveArmToPosition(ManualManager.ARM_MAX - 0.05);
        }
    }



}
