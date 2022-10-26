package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.skills.Gyroscope;
import org.firstinspires.ftc.teamcode.skills.Led;

public class EncoderBot {
    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;

    private int MAX_VELOCITY = 2000;

    protected double FRONT_LEFT_POWER_FACTOR = 0.5;
    protected double BACK_LEFT_POWER_FACTOR = 0.5;
    protected double FRONT_RIGHT_POWER_FACTOR = 0.5;
    protected double BACK_RIGHT_POWER_FACTOR = 0.5;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;
    protected LinearOpMode owner = null;

    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private BotCalibConfig botConfig;

    public static String LEFT_FRONT = "frontLeft";
    public static String RIGHT_FRONT = "frontRight";
    public static String LEFT_BACK = "backLeft";
    public static String RIGHT_BACK = "backRight";

    private static final String TAG = "EncoderBot";

    public EncoderBot() {

    }

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        try {
            frontLeft = hwMap.get(DcMotorEx.class, LEFT_FRONT);
            frontRight = hwMap.get(DcMotorEx.class, RIGHT_FRONT);
            backLeft = hwMap.get(DcMotorEx.class, LEFT_BACK);
            backRight = hwMap.get(DcMotorEx.class, RIGHT_BACK);

            resetEncoders();
            setUpMotors();
            stop();
        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing one of drive motors. Check the controller config", ex);
        }
    }

    protected void resetEncoders() {
        if (frontLeft != null) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (frontRight != null) {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (backLeft != null) {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (backRight != null) {
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stop() {
        if (frontLeft != null) {
            frontLeft.setPower(0);
        }
        if (frontRight != null) {
            frontRight.setPower(0);
        }
        if (backLeft != null) {
            backLeft.setPower(0);
        }
        if (backRight != null) {
            backRight.setPower(0);
        }
    }

    protected void setUpMotors() {
        if (backLeft != null) {
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backRight != null) {
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (frontLeft != null) {
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (frontRight != null) {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void move(double drive, double turn){
        double rightPower = Range.clip(drive + (turn * 0.85), -1.0, 1.0);
        double leftPower = Range.clip(drive - (turn * 0.85), -1.0, 1.0);
        if ((drive > 0 && drive <= 4 )|| (turn > 0 && turn <= 4)){
            rightPower = rightPower * rightPower * rightPower;
            leftPower = leftPower * leftPower * leftPower;
        }

        if (backLeft != null) {
            this.backLeft.setPower(leftPower * BACK_LEFT_POWER_FACTOR);
        }
        if (backRight != null) {
            this.backRight.setPower(rightPower * BACK_RIGHT_POWER_FACTOR);
        }
        if (frontLeft != null) {
            this.frontLeft.setPower(leftPower * FRONT_LEFT_POWER_FACTOR);
        }
        if (frontRight != null) {
            this.frontRight.setPower(rightPower * FRONT_RIGHT_POWER_FACTOR);
        }

        telemetry.addData("RUNNNNNNNNNING W/ ENCODERS", "YESSSSSSSSSS");
        telemetry.addData("Motors", "Left: %.0f", leftPower);
        telemetry.addData("Motors", "Right: %.0f", rightPower);
        telemetry.addData("Motors", "Turn: %.0f", turn);
        telemetry.addData("Motors", "LeftFront from %7d", frontLeft.getCurrentPosition());
        telemetry.addData("Motors", "LeftBack from %7d", backLeft.getCurrentPosition());
        telemetry.addData("Motors", "RightFront from %7d", frontRight.getCurrentPosition());
        telemetry.addData("Motors", "RightBack from %7d", backRight.getCurrentPosition());
    }
}
