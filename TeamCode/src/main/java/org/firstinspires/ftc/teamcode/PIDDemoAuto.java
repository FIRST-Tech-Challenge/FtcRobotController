package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        strafeToDistanceNoHeading(0.3, Math.PI/6, 0.2);
        robot.strafeToDistance(0.3, 4*Math.PI/6, 0.2);
        strafeToDistanceNoHeading(0.3, -7*Math.PI/6, 0.2);
        robot.strafeToDistance(0.3, -10*Math.PI/6, 0.2);
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
