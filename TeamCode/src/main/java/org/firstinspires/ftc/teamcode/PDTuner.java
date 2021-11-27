package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

/*
this opmode should tune the strafing PID's using a manual method
 */
@TeleOp(name="PD Tuner", group = "Testing")
//@Disabled
public class PDTuner extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    PIDController pid1       = new PIDController(0,0,0);
    PIDController pid2       = new PIDController(0,0,0);

    @Override
    public void runOpMode() {
        // declare some variables if needed

        double P=0,D=0;
        double scaleFactor = .001;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //PID config
        pid1.reset();
        pid1.setOutputRange(-1,1);
        pid1.setTolerance(5);
        pid1.setSetpoint(100);
        pid1.enable(runtime.seconds());

        pid2.reset();
        pid2.setOutputRange(-1,1);
        pid2.setTolerance(5);
        pid2.setSetpoint(100);
        pid2.enable(runtime.seconds());

        //button locks
        boolean upCurPressed, upPrevPressed = false,
                downCurPressed, downPrevPressed = false,
                leftCurPressed, leftPrevPressed = false,
                rightCurPressed, rightPrevPressed = false;
        boolean aCurPressed, aPrevPressed = false,
                bCurPressed, bPrevPressed = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //allow K to be adjusted with buttons
            upCurPressed = gamepad1.dpad_up;
            if (upCurPressed && !upPrevPressed ){
                P += scaleFactor;
            }
            upPrevPressed=upCurPressed;

            downCurPressed = gamepad1.dpad_down;
            if (downCurPressed && !downPrevPressed){
                P -= scaleFactor;
            }
            downPrevPressed=downCurPressed;

            leftCurPressed = gamepad1.dpad_left;
            if (leftCurPressed && !leftPrevPressed){
                D += scaleFactor;
            }
            leftPrevPressed=leftCurPressed;

            rightCurPressed = gamepad1.dpad_right;
            if (rightCurPressed && !rightPrevPressed){
                D -= scaleFactor;
            }
            rightPrevPressed=rightCurPressed;

            //update pid coefficients
            pid1.setPID(P,0,D);
            pid2.setPID(P,0,D);

            //move if button pressed
            aCurPressed = gamepad1.a;
            if (aCurPressed && !aPrevPressed) {
                strafeToDistanceNoHeading(1, Math.PI / 4.0, .3);
            }
            aPrevPressed=aCurPressed;
            bCurPressed = gamepad1.b;
            if (bCurPressed && !bPrevPressed) {
                strafeToDistanceNoHeading(1, (-Math.PI/4.0), .3);
            }
            bPrevPressed=bCurPressed;

            //telemetry
            telemetry.addData("scale factor: ", scaleFactor);
            telemetry.addLine("---====Coefficients====---");
            telemetry.addData("p: ", P);
            telemetry.addData("d: ", D);
            telemetry.update();

        }
    }
    /**
     * causes the robot to strafe a given direction, at a given power, for a given distance using PIDs
     * power and distance should always be positive
     * @param power             power factor
     * @param angle             direction to strafe relative to robot, measure in radians, angle of 0 == starboard
     * @param targetDistance    distance to strafe, measured in meters
     */
    public void strafeToDistanceNoHeading(double power, double angle, double targetDistance){
        //DATA
        //initial heading and encoder counts
        int initialFrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;
        double FrBlCorrection, FlBrCorrection;
        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)); //this is just done to help convert the radial inputs (magnitude and angle) into target distances that the PID can use
        double FlBrPairPower = Math.sin(angle + (Math.PI/4));
        //find the desired target for each strafe PID
        int FlBrAxisTarget = (int) ((targetDistance * (FlBrPairPower)) * robot.NADO_COUNTS_PER_METER);
        int FrBlAxisTarget = (int) ((targetDistance * (FrBlPairPower)) * robot.NADO_COUNTS_PER_METER);

        //set up the PIDs
        power = Math.abs(power);
        //PID
        pid1.reset();
        pid1.setSetpoint(FrBlAxisTarget+initialFrBlAxisEncoderCount);
        //pid1.setOutputRange(-power*FrBlPairPower,power*FrBlPairPower);
        pid1.enable(runtime.seconds());

        //FlBr PID
        pid2.reset();
        pid2.setSetpoint(FlBrAxisTarget+initialFlBrAxisEncoderCount);
        //pid2.setOutputRange(-power*FlBrPairPower,power*FlBrPairPower);
        pid2.enable(runtime.seconds());

        //run PID
        while ((!pid1.onTarget() && !pid2.onTarget()) && opModeIsActive()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
            int FlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

            //get motor power
            FrBlCorrection = pid1.performPID(FrBlAxisEncoderCount, runtime.seconds());
            FlBrCorrection = pid2.performPID(FlBrAxisEncoderCount, runtime.seconds());

            //assign drive motor powers
            robot.setDrivetrainPowerMecanum(
                    FrBlPairPower * FrBlCorrection,
                    FlBrPairPower * FlBrCorrection
            );

            if (gamepad1.x) {
                robot.setDrivetrainPower(0);
                break;
            }
        }
    }
}

