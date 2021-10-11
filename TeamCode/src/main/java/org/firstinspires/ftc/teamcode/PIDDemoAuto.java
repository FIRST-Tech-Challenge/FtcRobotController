package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "PID Demo Auto", group = "Off Season")
public class PIDDemoAuto extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // declare some variables if needed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //auto routine
        strafe(1, 0, .1);

        //strafeToDistanceNoHeading(1, Math.PI/2, 0.1);
        //robot.strafeToDistance(0.3, 4*Math.PI/6, 0.2);
        //strafeToDistanceNoHeading(0.3, -7*Math.PI/6, 0.2);
        //robot.strafeToDistance(0.3, -10*Math.PI/6, 0.2);
    }

    public void strafe(double power, double angle, double targetDistance){
        //initial heading and encoder counts
        int initialFrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)) * power;
        double FlBrPairPower = Math.sin(angle + (Math.PI/4)) * power;

        //find the desired target for each strafe PID
        // convert to encoder counts
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower/power) * robot.NADO_COUNTS_PER_METER); //in meters
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower/power) * robot.NADO_COUNTS_PER_METER); //in meters

        //set the power pairs to the absolute value of said powers
        FrBlPairPower = Math.abs(FrBlPairPower);
        FlBrPairPower = Math.abs(FlBrPairPower);

        //set up the PIDs
        //FrBl PID
        robot.FrBlStrafePIDController.reset();
        robot.FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        robot.FrBlStrafePIDController.enable();
        //FlBr PID
        robot.FlBrStrafePIDController.reset();
        robot.FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        robot.FlBrStrafePIDController.enable();

        //try doing the two movements separately before doing them at the same time
        while(!robot.FrBlStrafePIDController.onTarget() && opModeIsActive()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;

            //get correction values
            double FrBlCorrection = robot.FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount);

            //assign drive motor powers
            //robot.setDrivetrainPower(
            //        FrBlPairPower * FrBlCorrection,
            //        FlBrPairPower * 0,
            //        FrBlPairPower * FrBlCorrection,
            //        FlBrPairPower * 0
            //);

            telemetry.addData("target: ", FrBlAxisTarget);
            telemetry.addData("position: ", FrBlAxisEncoderCount);
            telemetry.addData("power: ", FrBlPairPower);
            telemetry.addData("correction: ", FrBlCorrection);
            telemetry.addData("error: ", robot.FrBlStrafePIDController.getError());
            telemetry.update();
        }
        while (!robot.FlBrStrafePIDController.onTarget() && opModeIsActive()) {
             //calculate distance travelled by each diagonal pair
            int FlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

            //get correction values
            double FlBrCorrection = robot.FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount);

            //assign drive motor powers
            //robot.setDrivetrainPower(
            //        FrBlPairPower * 0,
            //        FlBrPairPower * FlBrCorrection,
            //        FrBlPairPower * 0,
            //        FlBrPairPower * FlBrCorrection
            //);
            telemetry.addData("target: ", FlBrAxisTarget);
            telemetry.addData("position: ", FlBrAxisEncoderCount);
            telemetry.addData("power: ", FlBrPairPower);
            telemetry.addData("correction: ", FlBrCorrection);
            telemetry.addData("error: ", robot.FlBrStrafePIDController.getError());
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
        //initial heading and encoder counts
        int initialFrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)) * power;
        double FlBrPairPower = Math.sin(angle + (Math.PI/4)) * power;

        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower/power)); //in meters
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower/power)); //in meters
        // convert to encoder counts
        FrBlAxisTarget *= robot.NADO_COUNTS_PER_METER;
        FlBrAxisTarget *= robot.NADO_COUNTS_PER_METER;

        //set up the PIDs
        //FrBl PID
        robot.FrBlStrafePIDController.reset();
        robot.FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        robot.FrBlStrafePIDController.enable();
        //FlBr PID
        robot.FlBrStrafePIDController.reset();
        robot.FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        robot.FlBrStrafePIDController.enable();


        while ((!robot.FrBlStrafePIDController.onTarget() || !robot.FlBrStrafePIDController.onTarget()) && opModeIsActive()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
            int FlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

            //get correction values
            double FrBlCorrection = robot.FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount);
            double FlBrCorrection = robot.FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount);

            //assign drive motor powers
            robot.setDrivetrainPower(
                    FrBlPairPower * FrBlCorrection,
                    FlBrPairPower * FlBrCorrection,
                    FrBlPairPower * FrBlCorrection,
                    FlBrPairPower * FlBrCorrection
            );
        }
    }
}
