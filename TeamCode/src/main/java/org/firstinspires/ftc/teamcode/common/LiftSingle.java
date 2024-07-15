package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware_data.GoBilda435DcMotorData;

@Config
public class LiftSingle extends Component {
    private final DcMotorEx liftMotor;
    private final int positionTolerance = 10;
//    private final double maxVelocity = GoBilda435DcMotorData.maxTicksPerSec;
    private final int maxPos = 2000;
    private final int minPos = 0;
    private final int cruisingPos = 850;
    private final double defaultMaxPower = 0.90;
    private double maxPower = defaultMaxPower;
    private double holdPower = 0.75;

    public LiftSingle(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop() {
        stopAtPosition(liftMotor.getCurrentPosition());
    }

    public void up(double power) {
        if (!atTop(positionTolerance)) {
            setMotorPower(power);
        } else {
            stopAtPosition(maxPos);
        }
    }

    public void down(double power) {
        if (!atBottom(positionTolerance)) {
            setMotorPower(-power);
        } else {
            stopAtPosition(minPos);
        }
    }

    private boolean atTop(int tolerance) {
        if ((liftMotor.getCurrentPosition() - tolerance) >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    private boolean atBottom(int tolerance) {
        if ((liftMotor.getCurrentPosition() + tolerance) <= minPos) {
            return true;
        } else {
            return false;
        }
    }

    public void setMotorPower(double power) {
        liftMotor.setPower(power * maxPower);
    }

    public void goToTop() {
        liftMotor.setTargetPosition(maxPos);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(holdPower);
    }

    public void goToBottom() {
        liftMotor.setTargetPosition(minPos);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(holdPower);
    }

    public void goToCruisingPos() {
        liftMotor.setTargetPosition(cruisingPos);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(holdPower);
    }

    public void stopAtPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(holdPower);
    }

    public void log() {
        telemetry.addData("Position:  ", liftMotor.getCurrentPosition());
        telemetry.addData("Power:  ", liftMotor.getPower());
        telemetry.update();
    }

    public void update() {
    }
}