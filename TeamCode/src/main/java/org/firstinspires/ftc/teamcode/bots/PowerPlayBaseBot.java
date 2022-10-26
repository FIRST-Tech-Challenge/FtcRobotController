package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;

public class PowerPlayBaseBot {
    protected DampenedMotor frontLeft = null;
    protected DampenedMotor frontRight = null;
    protected DampenedMotor backLeft = null;
    protected DampenedMotor backRight = null;

    private int MAX_VELOCITY = 2000;

    protected double FRONT_LEFT_POWER_FACTOR = 1.0;
    protected double BACK_LEFT_POWER_FACTOR = 1.0;
    protected double FRONT_RIGHT_POWER_FACTOR = 1.0;
    protected double BACK_RIGHT_POWER_FACTOR = 1.0;

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

    private static final String TAG = "PowerPlayBaseBot";

    public PowerPlayBaseBot() {

    }

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        double dampeningFactor = 0.5;

        try {
            frontLeft = new DampenedMotor(hwMap.get(DcMotorEx.class, LEFT_FRONT), DcMotor.Direction.FORWARD, dampeningFactor);
            frontRight = new DampenedMotor(hwMap.get(DcMotorEx.class, RIGHT_FRONT), DcMotor.Direction.REVERSE, dampeningFactor);
            backLeft = new DampenedMotor(hwMap.get(DcMotorEx.class, LEFT_BACK), DcMotor.Direction.FORWARD, dampeningFactor);
            backRight = new DampenedMotor(hwMap.get(DcMotorEx.class, RIGHT_BACK), DcMotor.Direction.REVERSE, dampeningFactor);

            stop();
        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing one of drive motors. Check the controller config", ex);
        }
    }

    public void stop() {
        frontLeft.setTargetPower(0);
        frontRight.setTargetPower(0);
        backLeft.setTargetPower(0);
        backRight.setTargetPower(0);
    }

    public void move(double drive, double turn){
        double rightPower = Range.clip(drive + (turn * 0.85), -1.0, 1.0);
        double leftPower = Range.clip(drive - (turn * 0.85), -1.0, 1.0);
        if ((drive > 0 && drive <= 4 )|| (turn > 0 && turn <= 4)){
            rightPower = rightPower * rightPower * rightPower;
            leftPower = leftPower * leftPower * leftPower;
        }

        this.backLeft.setTargetPower(leftPower * BACK_LEFT_POWER_FACTOR);
        this.backRight.setTargetPower(rightPower * BACK_RIGHT_POWER_FACTOR);
        this.frontLeft.setTargetPower(leftPower * FRONT_LEFT_POWER_FACTOR);
        this.frontRight.setTargetPower(rightPower * FRONT_RIGHT_POWER_FACTOR);

        telemetry.addData("Dampened", "Running");
        telemetry.addData("Motors", "Left: %.0f", leftPower);
        telemetry.addData("Motors", "Right: %.0f", rightPower);
        telemetry.addData("Motors", "Turn: %.0f", turn);
        telemetry.addData("Motors", "LeftFront from %7d", frontLeft.getCurrentPosition());
        telemetry.addData("Motors", "LeftBack from %7d", backLeft.getCurrentPosition());
        telemetry.addData("Motors", "RightFront from %7d", frontRight.getCurrentPosition());
        telemetry.addData("Motors", "RightBack from %7d", backRight.getCurrentPosition());
    }
}