package org.firstinspires.ftc.teamcode.robots.conceptTrikeBot.utils;

import static org.firstinspires.ftc.teamcode.util.utilMethods.diffAngle2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name = "trike mixer test")
public class TrikeOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx swerve = null;
    private DcMotorEx swerveAngle = null;
    PIDController swervePID = null;

    double turnError = 0;

    public static double swerveP = .1;
    public static double swerveI = 0;
    public static double swerveD = 0;

    double vl = 0, vr = 0, v_swerve = 0;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        rightDrive = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        swerve  = hardwareMap.get(DcMotorEx.class, "motorMiddle");
        swerveAngle = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        swerve.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        swerveAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        swerve.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        swerveAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        swervePID = new PIDController(0,0,0);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(opModeIsActive()) {

            double forward = -gamepad1.right_stick_y;
            double rotate = gamepad1.left_stick_x;

            double length = Constants.CHASSIS_LENGTH, radius = 0, angle = 0;
            if (rotate == 0)
                angle = 0;
            else {
                radius = forward / rotate;
                angle = Math.atan2(length, radius);
            }

            vl = forward - rotate * (rotate == 0 ? 0 : radius - Constants.TRACK_WIDTH / 2);
            vr = forward + rotate * (rotate == 0 ? 0 : radius + Constants.TRACK_WIDTH / 2);
            v_swerve = forward + rotate * (rotate == 0 ? 0 : Math.hypot(radius, Constants.CHASSIS_LENGTH));

            double max = Math.max(Math.max(vl, vr), v_swerve);
            vl /= max;
            vr /= max;
            v_swerve /= max;

            leftDrive.setPower(vl);
            rightDrive.setPower(vr);
            swerve.setPower(v_swerve);

//            turnSwervePID(swerveP, swerveI, swerveD, angleOfBackWheel(), angle);

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
        telemetry.addData("swerve wheel velocity", v_swerve);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rotate", gamepad1.left_stick_x);
        packet.put("forward", -gamepad1.right_stick_y);
        packet.put("left wheel velocity", vl);
        packet.put("right wheel velocity", vr);
        packet.put("swerve wheel velocity", v_swerve);
        dashboard.sendTelemetryPacket(packet);
    }


    public void turnSwervePID(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {

        //initialization of the PID calculator's output range, target value and multipliers
        swervePID.setOutputRange(-.69, .69); //this is funny
        swervePID.setPID(Kp, Ki, Kd);
        swervePID.setSetpoint(targetAngle);
        swervePID.enable();

        //initialization of the PID calculator's input range and current value
        swervePID.setInputRange(0, 360);
        swervePID.setContinuous();
        swervePID.setInput(currentAngle);

        turnError = diffAngle2(targetAngle, currentAngle);

        //calculates the angular correction to apply
        double correction = swervePID.performPID();

        //performs the turn with the correction applied
//        swerveAngle.setPower(correction);
    }

    public double angleOfBackWheel(){
        return swerveAngle.getCurrentPosition() / Constants.swerveAngleTicksPerDegree;
    }
}
