package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

// ----- READY TO TRANFER ----- //

@Config
@TeleOp(name = "11Pid To Point Tuner",group = "Tuning")
public class PidToPointTuner extends LinearOpMode {

    Robot robot;

    public static double goalXININCHES = 0;
    public static double goalYININCHES = 0;
    public static double goalHINDEGREES = 0;

    public PIDController vertPid = new PIDController(0, 0, 0);
    public PIDController latPid = new PIDController(0, 0, 0);
    public PIDController turnPid = new PIDController(0, 0, 0);

    private EditablePose2D goal = new EditablePose2D(0,0, 0, DistanceUnit.INCH);

    private EditablePose2D start = new EditablePose2D(0,0,90, DistanceUnit.INCH);

    private double elapsedTime = 0;
    private double time = System.nanoTime() / (Math.pow(10, 9));

    private TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(MAX_A, MAX_V);

    private boolean dbounce = false;


    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();
        robot.getDrivetrain().startOdometry();

        goal = new EditablePose2D(robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH),
                robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH),
                robot.getDrivetrain().getCurrPos().getH(),
                DistanceUnit.INCH);

        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */
        while (opModeIsActive()) {
            tuneMethod();

            if(!gamepad1.a){
                dbounce = false;
            }
        }
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    double distance = 0;
    double lineSlope = 0;

    public void tuneMethod(){


        //find how much time has passed since the start of this path (seconds).
        double elapsedTime = System.nanoTime() / 1e9 - time;

        if (gamepad1.a && !dbounce){

            dbounce = true;

            //set old goal point as start point
            start.setX(robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH), DistanceUnit.INCH);
            start.setY(robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH), DistanceUnit.INCH);
            start.setH(robot.getDrivetrain().getCurrPos().getH());

            // set new goal Point
            goal.setX(goalXININCHES, DistanceUnit.INCH);
            goal.setY(goalYININCHES, DistanceUnit.INCH);
            goal.setH(Math.toRadians(goalHINDEGREES));

            //Find distance between two points for motion profile
            double dx = goal.getX(DistanceUnit.INCH) - start.getX(DistanceUnit.INCH);
            double dy = goal.getY(DistanceUnit.INCH) - start.getY(DistanceUnit.INCH);
            double distance = Math.sqrt(dy * dy + dx * dx);

            //Find the angle from the x axis for global coordinates - Used to find expected position in the future
            this.distance = distance;
            //Find the angle from the x axis for global coordinates - Used to find expected position in the future
            lineSlope = Math.toDegrees(Math.atan2(dy, dx));

            //Set the motion profile with the new calculated distance
            motionProfile.setProfile(MAX_A, MAX_V);
            motionProfile.calculateProfile(distance);

            //Clear pid integral windups
            latPid.resetIntegral();
            vertPid.resetIntegral();
            turnPid.resetIntegral();

            // reset the elapsed time for this path.
            time = System.nanoTime() / (Math.pow(10, 9));
            elapsedTime = 0;
        }

        //Find the current field positions
        double currentX = robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
        double currentY = robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);
        double currentH = robot.getDrivetrain().getCurrPos().getH();

        //Find the expected position along the path, as a magnitude of a vector with angle lineSlope
        // then use some trig to find x and y components
        double magnitude = motionProfile.getExpectedPosition(elapsedTime);
        double xComponent = start.getX(DistanceUnit.INCH) + magnitude * Math.cos(Math.toRadians(lineSlope));
        double yComponent = start.getY(DistanceUnit.INCH) + magnitude * Math.sin(Math.toRadians(lineSlope));

        //Set Pids to what they are supposed to be.a
        vertPid.setPID(VERT_KP, VERT_KI, VERT_KD);
        latPid.setPID(LAT_KP, LAT_KI, LAT_KD);
        turnPid.setPID(TURN_KP, TURN_KI, TURN_KD);

        //shift the errors to robot centric errors
        double xError = xComponent - currentX;
        double yError = yComponent - currentY;

        double xErrorRot = xError * Math.cos(-currentH) - yError * Math.sin(-currentH); // I could probably change this into -sin
        double yErrorRot = xError * Math.sin(-currentH) + yError * Math.cos(-currentH); // bc sin is odd but it would be too complicated
                                                                                        // to understand again if took a quick look
        //Find the forward and strafe powers
        double vertPower = vertPid.calculate(0, xErrorRot);
        double latPower = latPid.calculate(0, yErrorRot);

        //Find anglePower
        double angleError = getAngleError(Math.toDegrees(goal.getH()), Math.toDegrees(currentH));
        double anglePower = turnPid.calculate(0, angleError);

        //Set Wheel powers actually
        double denominator = Math.max(Math.abs(latPower) + Math.abs(vertPower) + Math.abs(anglePower), 1);
        double frWheelPower = (vertPower + latPower + anglePower) / denominator;
        double flWheelPower = (vertPower - latPower - anglePower) / denominator;
        double brWheelPower = (vertPower - latPower + anglePower) / denominator;
        double blWheelPower = (vertPower + latPower - anglePower) / denominator;

        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);

        //Ftc dashboard drawings
        double robotSize = 18;
        double headingLineLength = 12;
    }
}
