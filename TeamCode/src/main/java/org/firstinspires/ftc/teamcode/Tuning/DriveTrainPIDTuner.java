package org.firstinspires.ftc.teamcode.Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.riptideUtil;

// ----- READY TO TRANSFER ----- //


@TeleOp(name = "Drive Train PID Tuner", group = "Tuning")
public class DriveTrainPIDTuner extends LinearOpMode {

    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    //in IN
    public static double goalInINCHES = 0;
    public static double goalINDEGREES = 0;
    public static PIDController pid = new PIDController(0, 0, 0);
    private double lastGoal = 0;
    private double startPos = 0;
    private double elapsedTime;
    private double time = System.nanoTime() / (Math.pow(10, 9));

    private TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(0, 0);

    public enum Direction{
        VERT,
        LAT,
        TURN
    }

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

            Direction d = tuneDirection();
            t.addData("Direction:", d);
            t.addData("Goal", goalInINCHES);
            t.addData("Goal Angle", goalINDEGREES);
            t.addData("StartPos", startPos);
            t.update();
            telemetry.addLine("OP mode is active");
            telemetry.update();

        }
    }

    /**
     * @param targetAngle the angle we want to reach in deg.
     * @param currentAngle the angle that we are at in deg.
     * @return the direction of where to turn the angle which is the smallest.
     */
    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    Direction d = Direction.VERT;

    public Direction tuneDirection(){
        if (lastGoal != goalInINCHES) {
            startPos = lastGoal;
            lastGoal = goalInINCHES;
            motionProfile.calculateProfile(goalInINCHES, startPos);
            time = System.nanoTime() / Math.pow(10, 9);
            elapsedTime = 0;
        } else {
            elapsedTime = (System.nanoTime() / Math.pow(10, 9)) - time;
        }
        if(gamepad1.dpad_up){
            d = Direction.VERT;
            pid.resetIntegral();
            motionProfile.setProfile(riptideUtil.MAX_A_VERT, riptideUtil.MAX_V_VERT);
            motionProfile.calculateProfile(goalInINCHES, startPos);
            time = System.nanoTime() / Math.pow(10, 9);
            elapsedTime = 0;
        }

        if(gamepad1.dpad_right){
            d = Direction.LAT;
            pid.resetIntegral();
            motionProfile.setProfile(riptideUtil.MAX_A_LAT, riptideUtil.MAX_V_LAT);
            motionProfile.calculateProfile(goalInINCHES, startPos);
            time = System.nanoTime() / Math.pow(10, 9);
            elapsedTime = 0;
        }

        if(gamepad1.dpad_left){
            d = Direction.TURN;
            pid.resetIntegral();
        }
        double currentPos;
        double power;
        double expPos;
        switch(d){
            case TURN:
                double currentAngle = Math.toDegrees(robot.getDrivetrain().getCurrPos().getH());

                t.addData("Current Angle: ", currentAngle);

                double error = getAngleError(goalINDEGREES, currentAngle);


                pid.setPID(riptideUtil.TURN_KP, riptideUtil.TURN_KI, riptideUtil.TURN_KD);
                power = pid.calculate(0, error) ;
                power = Math.abs(power) > riptideUtil.MAX_WHEEL_POWER ? Math.signum(power) * riptideUtil.MAX_WHEEL_POWER : power;
                t.addData("Current Power * 50", power * 50);


                robot.getDrivetrain().setWheelPowers(power, -power, -power, power);

                break;
            case LAT:
                currentPos = robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);
                t.addData("Current Pos: ", currentPos);

                pid.setPID(riptideUtil.LAT_KP, riptideUtil.LAT_KI, riptideUtil.LAT_KD);
                expPos = startPos +  motionProfile.getExpectedPosition(elapsedTime);
                power = pid.calculate(currentPos, expPos);
                power = Math.abs(power) > riptideUtil.MAX_WHEEL_POWER ? Math.signum(power) * riptideUtil.MAX_WHEEL_POWER : power;

                t.addData("Expected Position - Lateral", expPos);
                t.addData("Current Power * 50", power * 50);

                robot.getDrivetrain().setWheelPowers(-power, power, -power, power);
                break;
            case VERT:
                currentPos = robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
                t.addData("Current Pos: ", currentPos);

                pid.setPID(riptideUtil.VERT_KP, riptideUtil.VERT_KI, riptideUtil.VERT_KD);
                expPos = startPos +  motionProfile.getExpectedPosition(elapsedTime);
                power = pid.calculate(currentPos, expPos);
                power = Math.abs(power) > riptideUtil.MAX_WHEEL_POWER ? Math.signum(power) * riptideUtil.MAX_WHEEL_POWER : power;

                t.addData("Expected Position - Vertical", expPos);
                t.addData("Current Power * 50", power * 50);

                robot.getDrivetrain().setWheelPowers(power, power, power, power);
                break;


        }


        return d;
    }
}
