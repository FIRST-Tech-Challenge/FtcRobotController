package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class potatoTurret {
    private DcMotorEx turret;

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;

    private final double MAX_POWER = 0.6;

    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // init the motors
    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // getters and setters
    public void setKP(double newKp) {
        kP = newKp;
    }

    public void setKD(double newKD) {
        kD = newKD;
    }

    public double getKd() {
        return kD;
    }

    public double getKp() {
        return kP;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(LLResultTypes.FiducialResult curId) {
        // set our delta time and reest our timer every update
        double dT = timer.seconds();
        timer.reset();

        // because we don't have a valid id, its okay to turn off and break
        if (curId == null) {
            turret.setPower(0);
            lastError = 0;
            // break out the method
            return;
        }

        double error = goalX - curId.getTargetXDegrees();
        double pTerm = error * kP;

        double dTerm = 0;
        if (dT > 0) {
            dTerm = ((error - lastError) / dT) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);

        lastError = error;
    }
}
