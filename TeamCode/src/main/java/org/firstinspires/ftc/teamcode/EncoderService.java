package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderService {
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    private RobotHardware robot;
    private ElapsedTime runtime = new ElapsedTime();

    public EncoderService(RobotHardware robotHardware){
        robot = robotHardware;
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        //// Ensure that the OpMode is still active
        //if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getLeftMotorCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getRightMotorCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.setLeftTargetPosition(newLeftTarget);
            robot.setRightTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setDrivePower(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
                    (robot.isLeftMotorBusy() && robot.isRightMotorBusy())) {

                // Display it for the drivAutoDriveByEncoderer.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.getLeftMotorCurrentPosition(), robot.getRightMotorCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setDrivePower(0, 0);

            // Turn off RUN_TO_POSITION
            robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
}
