package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;


// ----- READY TO TRANSFER ----- //

@Config
@TeleOp(name = "ToTuneLateralStrafingThisWasCreated", group = "Tuning")
public class LateralTuning extends LinearOpMode {

    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard d = FtcDashboard.getInstance();


    public static double nextGoalXInInches = 0;
    public static double nextGoalYInInches = 0;
    public static double nextGoalHInDegrees = 0;


    private  double goalXININCHES = 0;
    private  double goalYININCHES = 0;
    private  double goalHINDEGREES = 0;

    //Visual constants


    // GOAL POSITION
    public EditablePose2D goal = new EditablePose2D(goalXININCHES, goalYININCHES, Math.toRadians(goalHINDEGREES), DistanceUnit.INCH);
    public static PIDController turnPid = new PIDController(0, 0, 0);
    public static PIDController vertPid = new PIDController(0, 0, 0);
    public static PIDController latPid = new PIDController(0, 0, 0);
    private EditablePose2D lastGoal = new EditablePose2D(0, 0, 0, DistanceUnit.INCH);
    private EditablePose2D startPos = new EditablePose2D(0, 0, 0, DistanceUnit.INCH);
    private double elapsedTime;
    private double time = System.nanoTime() / (Math.pow(10, 9));

    private TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(0, 0);

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
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */
        while (opModeIsActive()) {
            updateGoal();
            goal.setX(goalXININCHES, DistanceUnit.INCH);
            goal.setY(goalYININCHES, DistanceUnit.INCH);
            goal.setH(Math.toRadians(goalHINDEGREES));

            TelemetryPacket f = tuneMovement(new TelemetryPacket());
            
            f.put("GoalX", goal.getX(DistanceUnit.INCH) );
            f.put("goal Y", goal.getY(DistanceUnit.INCH));
            f.put("Goal Angle", goal.getH());
            d.sendTelemetryPacket(f);
            telemetry.addLine("OP mode is active");
            telemetry.update();

        }
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }
    
    public void updateGoal(){
        if(gamepad1.dpad_up){
            goalXININCHES = nextGoalXInInches;
            goalYININCHES = nextGoalYInInches;
            goalHINDEGREES = nextGoalHInDegrees;
        }
    }

    public TelemetryPacket tuneMovement(TelemetryPacket p) {

        double pathAngle = 0;

        if (!lastGoal.equals(goal)) {
            startPos.setX(lastGoal.getX(DistanceUnit.INCH), DistanceUnit.INCH);
            startPos.setY(lastGoal.getY(DistanceUnit.INCH), DistanceUnit.INCH);
            startPos.setH(lastGoal.getH());
            lastGoal.setX(goal.getX(DistanceUnit.INCH), DistanceUnit.INCH);
            lastGoal.setY(goal.getY(DistanceUnit.INCH), DistanceUnit.INCH);
            lastGoal.setH(goal.getH());

            double dx = goal.getX(DistanceUnit.INCH) - startPos.getX(DistanceUnit.INCH);
            double dy = goal.getY(DistanceUnit.INCH) - startPos.getY(DistanceUnit.INCH);

            double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            pathAngle = Math.atan2(dy, dx);
            motionProfile.setProfile(MAX_A, MAX_V);
            motionProfile.calculateProfile(distance);

            time = System.nanoTime()/1e9;
            elapsedTime = 0;
        }
        else{
            elapsedTime = System.nanoTime()/1e9 - time;
        }

        double currentAngleRadians = robot.getDrivetrain().getCurrPos().getH();
        double currentAngle = Math.toDegrees(currentAngleRadians);
        double currentX = robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
        double currentY = robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);

//        currentX = currentX * Math.cos( - currentAngleRadians) - currentY * Math.sin(-currentAngleRadians);
//        currentY = currentX * Math.sin(-currentAngleRadians) + currentY * Math.cos(-currentAngleRadians);

        p.put("Current Angle: ", currentAngle);
        p.put("Current X", currentX);
        p.put("Current Y", currentY);

        double turnError = getAngleError(goalHINDEGREES, currentAngle);
        double linearExpPos = motionProfile.getExpectedPosition(elapsedTime);
        double expX = startPos.getX(DistanceUnit.INCH) + Math.cos(pathAngle) * linearExpPos;
        double expY = startPos.getY(DistanceUnit.INCH) + Math.sin(pathAngle) * linearExpPos;

        p.put("exp x", expX);
        p.put("exp y", expY);

        turnPid.setPID(TURN_KP, riptideUtil.TURN_KI, riptideUtil.TURN_KD);
        vertPid.setPID(VERT_KP, riptideUtil.VERT_KI, VERT_KD);
        latPid.setPID(LAT_KP, LAT_KI, LAT_KD);

        double turnPower = turnPid.calculate(0, turnError);
        double xPower = vertPid.calculate(currentX, expX);
        double yPower = latPid.calculate(currentY, expY);

        p.put("turnPower * 50 ", turnPower * 50);
        p.put("power Y * 50", yPower * 50);
        p.put("power X * 50", xPower * 50);

        double xPowerRotated = xPower * Math.cos(-currentAngleRadians) - yPower * Math.sin(-currentAngleRadians);
        double yPowerRotated = xPower * Math.sin(-currentAngleRadians) + yPower * Math.cos(-currentAngleRadians);

        double denominator = Math.max(Math.abs(yPowerRotated) + Math.abs(xPowerRotated) + Math.abs(turnPower), 1);
        double frWheelPower = (yPowerRotated - xPowerRotated - turnPower) / denominator;
        double flWheelPower = (yPowerRotated + xPowerRotated + turnPower) / denominator;
        double brWheelPower = (yPowerRotated + xPowerRotated - turnPower) / denominator;
        double blWheelPower = (yPowerRotated - xPowerRotated + turnPower) / denominator;

        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);


        //Visual effects

        int robotSize = 18;
//
//        p.fieldOverlay().drawGrid( -72,-72, 144, 144, 7, 7)
//                .setTranslation(36, 72 - (double) robotSize /2)
//                .setFill("Blue")
//                .setRotation(currentAngleRadians)
//                .fillRect(currentY - (double) robotSize /2, currentX - (double) robotSize /2, robotSize, robotSize)
//                .setFill("Red")
//                .setRotation(Math.toRadians(goalHINDEGREES))
//                .fillRect(expY - (double) robotSize /2, expX - (double) robotSize /2, robotSize, robotSize)
//                .setFill("Red")
//                .fillCircle(nextGoalXInInches, nextGoalYInInches, 2)
//                .setFill("Blue")
//                .fillCircle(startPos.getX(DistanceUnit.INCH), startPos.getY(DistanceUnit.INCH), 2);

        return p;

    }
}
