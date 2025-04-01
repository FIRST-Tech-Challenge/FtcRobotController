package org.firstinspires.ftc.team13580.TeleOp;


/*
* This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
* This approach is very difficult because the sme hardware class can be used by all of our teleop and autonomous
* OpModes without requiring many copy and paste operations. Once We define and test the hardware class with one OpMode,
* It will instantly be available to other OpModes.
*
* The real benefit of this approach is that as we tweak
*  our robot hardware, we'd only need to make changes in ONE place
*  (the Hardware Class).
*
* The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
* In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
* OpMode object when it's created, so it can access all core OpMode functions.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name="Concept: Robot Hardware Class", group="Robot")

public class RobotCentric extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    //Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){
        double axial      = 0;
        double lateral    = 0;
        double yaw        = 0;
        //double upDown        = 0;
        double handOffset = 0;
        double upDown = 0;
        double spoolie = 0;
        double armPositionFudgeFactor;
        double hangPositionFudgeFactor;
        double hang=0;
        double elbowHang=0;
        double armPosition = robot.ARM_COLLAPSED_INTO_ROBOT;

        //Initialize all hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        //Send telemetry message to signify robot waiting;
        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            // since the joystick goes negative when pushed forward, we negate it
            //In this mode the Left stick moves the robot forward and back, the Right stick turns L and R.
            //This way it's also easy to just drive straight, or just turn.
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x * 1.1;
            yaw= gamepad1.right_stick_x;

            // combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobotCentric(axial, lateral, yaw);

            if (gamepad1.right_bumper) {
                handOffset += robot.HAND_SPEED;
            } else if (gamepad1.left_bumper) {
                handOffset -= robot.HAND_SPEED;
            }
            handOffset = Range.clip(handOffset, -0.1, 0.5);
            //passes the positions of the hand to the robotHardware class without this line it will not move
            robot.setHandPositions(handOffset);

            //passes the position of the elbow of the robot(the motor in the arm, not the spoolie)
            //to up when pressing the right bumper in the game controller 2
            // ad to down when pressing the left bumper of gam controller 2
            if (gamepad2.right_bumper) {
                spoolie = robot.ARM_UP_POWER;
            } else if (gamepad2.left_bumper) {
                spoolie = robot.ARM_DOWN_POWER;
            } else {
                spoolie = 0;
            }
            robot.setSpooliePower(spoolie);

            if (gamepad1.x) {
                hang = robot.ARM_UP_POWER;
            } else if (gamepad1.y) {
                hang = robot.ARM_DOWN_POWER;
            } else {
                hang = 0;
            }
            robot.setHangPower(hang);

            //position of the arm of the robot using encoders
            if (gamepad2.a) {
                armPosition = robot.ARM_COLLAPSED_INTO_ROBOT;
            } else if (gamepad2.b) {
                armPosition = robot.ARM_ATTACH_HANGING_HOOK;
            } else if (gamepad2.x) {
                armPosition = robot.ARM_CLEAR_BARRIER;
            } else if (gamepad2.y) {
                armPosition = robot.ARM_SCORE_SAMPLE_IN_LOW;
            }else if (gamepad2.dpad_down){
                armPosition= robot.ARM_SECURE_SPECIMEN;
            }else if(gamepad2.dpad_up){
                armPosition = robot.ARM_SCORE_SPECIMEN;
            }else if(gamepad2.dpad_left){
                armPosition= robot.ARM_SCORE;
            }else if(gamepad2.dpad_right){
                armPosition= robot.ARM_SPECIMEN;
            }else if(gamepad2.left_stick_button){
                armPosition=robot.ARM_COLLECT;
            }
            armPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            robot.upDown.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            ((DcMotorEx)robot.upDown).setVelocity(2100);
            robot.upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.a){
                elbowHang= robot.HANG_UP;
            }else if(gamepad1.b){
                elbowHang= robot.HANG_COLLAPSED_INTO_ROBOT;
            }
            hangPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            robot.elbowHang.setTargetPosition((int) (elbowHang + hangPositionFudgeFactor));
            ((DcMotorEx)robot.elbowHang).setVelocity(1000);
            robot.elbowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
