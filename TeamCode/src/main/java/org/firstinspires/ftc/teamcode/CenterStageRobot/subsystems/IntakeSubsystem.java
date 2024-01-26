package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorExEx motor;
    public double speed = 0.9;  // TODO: Speed Value Might Change
    private final double ampThreshold = 1.4;
    private Telemetry telemetry;

    private boolean isStalled = false;
    private final Timing.Timer timer;

    public IntakeSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.motor = new MotorExEx(hm, "intake");
        motor.setInverted(true);

        this.telemetry = telemetry;

        this.timer = new Timing.Timer(2500, TimeUnit.MILLISECONDS);
    }

    public double getCurrent() {
        return ((DcMotorEx)motor.getRawMotor()).getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Amps: ", getCurrent());
//        if(isStalled()) {
//            telemetry.addData("Stalled", "");
//            isStalled = false;
//        } else {
//            telemetry.addData("not Stalled", "");
//        }

        if(getCurrent() > ampThreshold && !timer.isTimerOn()) {
            timer.start();
        }

        if(timer.done() && getCurrent() > ampThreshold){
            isStalled = true;
        }
    }

    public void run() {
        motor.set(speed);
    }

    public void run_auto() {
        motor.set(0.85);
    }

    public void reverse() {
        motor.set(-speed);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.set(0);
    }

    public void slow_grabbing() {
        motor.set(0.35);
    }

    public boolean isStalled() {
        return isStalled;
    }
}
