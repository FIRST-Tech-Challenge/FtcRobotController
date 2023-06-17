package org.firstinspires.ftc.teamcode.robots.reach.utils;

import static org.firstinspires.ftc.teamcode.util.utilMethods.diffAngle2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Disabled
@TeleOp(name = "trike mixer test")
public class TrikeOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx swerve = null;
    private DcMotorEx swerveAngle = null;
    private DistanceSensor distanceSensor = null;

    double turnError = 0;

    PIDController swervePID = null;
    public static double swerveP = .1;
    public static double swerveI = 0;
    public static double swerveD = 0;
    public double correction = 0;

    PIDController distancePID = null;

    double vl = 0, vr = 0, vSwerve = 0, angleSwerve = 0;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        rightDrive = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        swerve  = hardwareMap.get(DcMotorEx.class, "motorMiddle");
        swerveAngle = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distLength");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        swerve.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        swerveAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        swerve.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        swerveAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        swerve.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        swerveAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        swervePID = new PIDController(0,0,0);
        distancePID = new PIDController(Constants.distanceP, Constants.distanceI, Constants.distanceD);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(opModeIsActive()) {

            double forward = -gamepad1.right_stick_y;
            double rotate = gamepad1.left_stick_x;

            double length = Constants.CHASSIS_LENGTH, radius = 0;
            if (rotate == 0)
                angleSwerve = 0;
            else {
                radius = forward / rotate;
                angleSwerve = Math.atan2(length, radius);
            }

            vl = forward + rotate * (rotate == 0 ? 0 : radius - Constants.TRACK_WIDTH / 2);
            vr = forward + rotate * (rotate == 0 ? 0 : radius + Constants.TRACK_WIDTH / 2);
            vSwerve = forward + rotate * (rotate == 0 ? 0 : Math.hypot(radius, Constants.CHASSIS_LENGTH));
            vl *= Constants.MOVEMENT_MULTIPLIER;
            vr *= Constants.MOVEMENT_MULTIPLIER;
            vSwerve *= Constants.MOVEMENT_MULTIPLIER;

            // converting from tangential m/s to angular ticks/s
            vl = vl / (2 * Math.PI * Constants.wheelRadius) * Constants.DRIVETRAIN_TICKS_PER_REVOLUTION;
            vr = vr / (2 * Math.PI * Constants.wheelRadius) * Constants.DRIVETRAIN_TICKS_PER_REVOLUTION;
            vSwerve = vSwerve / (2 * Math.PI * Constants.wheelRadius) * Constants.DRIVETRAIN_TICKS_PER_REVOLUTION;

            double max = Math.max(Math.max(vl, vr), vSwerve);

            if(max > 1.0) {
                vl /= max;
                vr /= max;
                vSwerve /= max;
            }

            double correction = getMaintainDistanceCorrection();

            leftDrive.setVelocity(vl - correction);
            rightDrive.setVelocity(vr - correction);
            swerve.setVelocity(vSwerve);

            turnSwervePID(swerveP, swerveI, swerveD, angleOfBackWheel(), angleSwerve);

            sendTelemetry();
        }
    }

//    private void metersToTicksPerSecond(double metersPerSecond) {
//        return metersPerSecond / ()
//    }

    private void sendTelemetry() {
        telemetry.addData("rotate", gamepad1.left_stick_x);
        telemetry.addData("forward", -gamepad1.right_stick_y);
        telemetry.addData("left wheel velocity", vl);
        telemetry.addData("right wheel velocity", vr);
        telemetry.addData("swerve wheel velocity", vSwerve);
        telemetry.addData("swerve wheel angle", angleSwerve);
        telemetry.addData("swerve wheel correction", correction);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rotate", gamepad1.left_stick_x);
        packet.put("forward", -gamepad1.right_stick_y);
        packet.put("left wheel velocity", vl);
        packet.put("right wheel velocity", vr);
        packet.put("swerve wheel velocity", vSwerve);
        packet.put("swerve wheel angle", angleSwerve);
        packet.put("swerve wheel correction", correction);
        dashboard.sendTelemetryPacket(packet);
    }


    public void turnSwervePID(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {

        //initialization of the PID calculator's output range, target value and multipliers
        swervePID.setOutputRange(-.69, .69); //this is funny
        swervePID.setPID(Kp, Ki, Kd);
        swervePID.setSetpoint(Math.toRadians(targetAngle));
        swervePID.enable();

        //initialization of the PID calculator's input range and current value
        swervePID.setInputRange(0, 360);
        swervePID.setContinuous();
        swervePID.setInput(currentAngle);

        turnError = diffAngle2(targetAngle, currentAngle);

        //calculates the angular correction to apply
        correction = swervePID.performPID();

        //performs the turn with the correction applied
        swerveAngle.setPower(correction);
    }

    public double getMaintainDistanceCorrection() {
        distancePID.setOutputRange(-0.5, 0.5);
        distancePID.setPID(Constants.distanceP, Constants.distanceI, Constants.distanceD);
        distancePID.setSetpoint(Constants.TARGET_DISTANCE);
        distancePID.enable();

        distancePID.setInputRange(0, 0.6);
        distancePID.setInput(getChassisLength());

        correction = distancePID.performPID();
        return correction;
    }

    public double getChassisLength() {
        return distanceSensor.getDistance(DistanceUnit.METER);
    }

    public double angleOfBackWheel(){
        return swerveAngle.getCurrentPosition() / Constants.swerveAngleTicksPerDegree;
    }
}
