package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveBase {
    private ElapsedTime runtime = new ElapsedTime();
    private static DcMotor.RunMode runmode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;
    public double leftPowerFront  = 1.0;
    public double rightPowerFront = 1.0;
    public double rightPowerBack  = 1.0;
    public double leftPowerBack   = 1.0;
    public double speedFactor     = 1.0;

    public MecanumDriveBase(HardwareMap hardwareMap){
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");

        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        // Run Without Encoders
        lf.setMode(runmode);
        rf.setMode(runmode);
        lf.setMode(runmode);
        rf.setMode(runmode);
        // Brake when power set to Zero
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
      public void gamepadController(Gamepad gamepad) {

          double drive = -gamepad.left_stick_y;
          double turn = gamepad.right_stick_x;
          double strafe = gamepad.left_stick_x;
          speedFactor = .5 + .5 * gamepad.right_trigger;
          driveMotors(drive, turn, strafe, speedFactor);
      }
      public void driveMotors(double drive,double turn,double strafe,double speedFactor){

          leftPowerFront  = (drive + turn + strafe) * speedFactor;
          rightPowerFront = (drive - turn - strafe) * speedFactor;
          leftPowerBack   = (drive + turn - strafe) * speedFactor;
          rightPowerBack  = (drive - turn + strafe) * speedFactor;

          lf.setPower(leftPowerFront);
          rf.setPower(rightPowerFront);
          lb.setPower(leftPowerBack);
          rb.setPower(rightPowerBack);
      }
      public void driveBaseTelemetry(Telemetry telemetry) {
        telemetry.addData("Motors", "lf(%.2f), rf(%.2f), lb(%.2f), rb(%.2f)", leftPowerFront, rightPowerFront, leftPowerBack, rightPowerBack);
        telemetry.addData("Speed control", speedFactor);
      }

    public DcMotor getlf() {
        return lf;
    }

    public DcMotor getlb() {
        return lb;
    }

    public DcMotor getrb() {
        return rb;
    }

    public DcMotor getrf() {
        return rf;
    }

    public double getLeftPowerFront() {
        return leftPowerFront;
    }

    public double getRightPowerFront() {
        return rightPowerFront;
    }

    public double getRightPowerBack() {
        return rightPowerBack;
    }

    public double getLeftPowerBack() {
        return leftPowerBack;
    }

    public double getSpeedFactor() {
        return speedFactor;
    }
}

