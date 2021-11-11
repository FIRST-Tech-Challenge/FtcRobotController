package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "PID Demo Auto", group = "Off Season")
@Disabled
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
        strafeToDistanceNoHeading(1,Math.PI,.2);
        //strafeToDistanceNoHeading(1, Math.PI/2.0, 0.1);
        //robot.strafeToDistance(0.3, 4.0*Math.PI/6.0, 0.2);
        //strafeToDistanceNoHeading(0.3, -7.0*Math.PI/6.0, 0.2);
        //robot.strafeToDistance(0.3, -10.0*Math.PI/6.0, 0.2);
    }

    public void strafe(double power, double angle, double targetDistance){
        //DATA
        //initial heading and encoder counts
        int initialFrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4));
        double FlBrPairPower = Math.sin(angle + (Math.PI/4));
        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower) * robot.NADO_COUNTS_PER_METER); //in encoder counts
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower) * robot.NADO_COUNTS_PER_METER); //in encoder counts

        //set up the PIDs
        power = Math.abs(power);
        //FrBl PID
        robot.FrBlStrafePIDController.reset();
        robot.FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        robot.FrBlStrafePIDController.setOutputRange(-power*FrBlPairPower,power*FrBlPairPower);
        robot.FrBlStrafePIDController.enable(runtime.seconds());
        //FlBr PID
        robot.FlBrStrafePIDController.reset();
        robot.FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        robot.FlBrStrafePIDController.setOutputRange(-power*FrBlPairPower,power*FrBlPairPower);
        robot.FlBrStrafePIDController.enable(runtime.seconds());

        //try doing the two movements separately before doing them at the same time
        while(!robot.FrBlStrafePIDController.onTarget() && opModeIsActive()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;

            //get correction values
            /*double FrBlCorrection*/FrBlPairPower = robot.FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount, runtime.seconds());

            //assign drive motor powers
            robot.setDrivetrainPower(
                    FrBlPairPower,
                    0,
                    FrBlPairPower,
                    0
            );

            telemetry.addData("target: ", FrBlAxisTarget);
            telemetry.addData("position: ", FrBlAxisEncoderCount);
            telemetry.addData("power: ", FrBlPairPower);
            telemetry.addData("error: ", robot.FrBlStrafePIDController.getError());
            telemetry.update();
        }
        while (!robot.FlBrStrafePIDController.onTarget() && opModeIsActive()) {
             //calculate distance travelled by each diagonal pair
            int FlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

            //get correction values
            /*double FlBrCorrection*/FlBrPairPower = robot.FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount,runtime.seconds());

            //assign drive motor powers
            robot.setDrivetrainPower(
                    0,
                    FlBrPairPower,
                    0,
                    FlBrPairPower
            );
            telemetry.addData("target: ", FlBrAxisTarget);
            telemetry.addData("position: ", FlBrAxisEncoderCount);
            telemetry.addData("power: ", FlBrPairPower);
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
        //DATA
        //initial heading and encoder counts
        int initialFrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
        int initialFlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

        //calculate desired power for each diagonal motor pair
        double FrBlPairPower = Math.sin(angle - (Math.PI/4)); //this is just done to help convert the radial inputs (magnitude and angle) into target distances that the PID can use
        double FlBrPairPower = Math.sin(angle + (Math.PI/4));
        //find the desired target for each strafe PID
        int FrBlAxisTarget = (int) (targetDistance * (FrBlPairPower)); //in meters
        int FlBrAxisTarget = (int) (targetDistance * (FlBrPairPower)); //in meters
        // convert to encoder counts
        FrBlAxisTarget *= robot.NADO_COUNTS_PER_METER;
        FlBrAxisTarget *= robot.NADO_COUNTS_PER_METER;

        //set up the PIDs
        power = Math.abs(power);
        //FrBl PID
        robot.FrBlStrafePIDController.reset();
        robot.FrBlStrafePIDController.setSetpoint(FrBlAxisTarget);
        robot.FrBlStrafePIDController.setOutputRange(-power*FrBlPairPower,power*FrBlPairPower);
        robot.FrBlStrafePIDController.enable(runtime.seconds());
        //FlBr PID
        robot.FlBrStrafePIDController.reset();
        robot.FlBrStrafePIDController.setSetpoint(FlBrAxisTarget);
        robot.FlBrStrafePIDController.setOutputRange(-power*FlBrPairPower,power*FlBrPairPower);
        robot.FlBrStrafePIDController.enable(runtime.seconds());

        //run PID
        while ((!robot.FrBlStrafePIDController.onTarget() || !robot.FlBrStrafePIDController.onTarget()) && opModeIsActive()) {
            //calculate distance travelled by each diagonal pair
            int FrBlAxisEncoderCount = (robot.driveFrontRight.getCurrentPosition() + robot.driveBackLeft.getCurrentPosition())/2;
            int FlBrAxisEncoderCount = (robot.driveFrontLeft.getCurrentPosition() + robot.driveBackRight.getCurrentPosition())/2;

            //get motor power
            /*double FrBlCorrection*/ FrBlPairPower = robot.FrBlStrafePIDController.performPID(FrBlAxisEncoderCount - initialFrBlAxisEncoderCount, runtime.seconds());
            /*double FlBrCorrection*/ FlBrPairPower = robot.FlBrStrafePIDController.performPID(FlBrAxisEncoderCount - initialFlBrAxisEncoderCount, runtime.seconds());

            //assign drive motor powers
            robot.setDrivetrainPower(
                    FrBlPairPower,// * FrBlCorrection,
                    FlBrPairPower,// * FlBrCorrection,
                    FrBlPairPower,// * FrBlCorrection,
                    FlBrPairPower// * FlBrCorrection
            );
        }
    }
}
