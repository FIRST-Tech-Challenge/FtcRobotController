package org.firstinspires.ftc.teamcode.Utils.Shooting;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;
import org.firstinspires.ftc.teamcode.Utils.Movement.PosGetters.PositionGetter;
import org.firstinspires.ftc.teamcode.Utils.Positioning.DistanceToHive;


/** In April tags turret target you are most likely going to be using:
 * - Update tx values
 * - Change Angle set position
 * - Check position and set power
 **/
public class AprilTagsTurretTarget {
    // Limelight Pipelines
    // 3 Blue
    // 4 Red

    private final ElapsedTime dtTimer = new ElapsedTime();
    double lastTx = 0;

    DcMotorEx dcMotorEx;

    int limelightPipeline;
    private long lastPipelineSwitchMs = 0;

    public double maxPosition = MiscUtils.getPositionFromDegrees(185, 537.7); //185
    public double minPosition = MiscUtils.getPositionFromDegrees(-185,537.7);
    public boolean waitTillNotSee = false;
    public boolean canSee = false;

    boolean didLastTX = false;

    double newPosition = 0;

    long seeTime;

    double previousHeading;
    PositionGetter positionGetter;

    public double offset;

    public boolean rightSide = true;

    DistanceToHive distanceToHive;

    public AprilTagsTurretTarget(DcMotorEx turretMotor, PositionGetter positionGetter, double offset){
        dcMotorEx = turretMotor;

        limelightPipeline = LinearOpMode2026.isCurrentAllienceRed ? 4 : 3;

        this.positionGetter = positionGetter;
        this.offset = offset;
        this.distanceToHive = new DistanceToHive(positionGetter);
    }

    public double UpdateTxValue(Limelight3A limelight3A){
        LLResult limelightResult = limelight3A.getLatestResult();

        // getStatus() is a blocking HTTP request and pipelineSwitch() a blocking POST (15 s timeout),
        // so read the pipeline from the latest result instead, and only switch when connected, max once a second.
        long now = System.currentTimeMillis();
        if (limelight3A.isConnected() && limelightResult != null
                && limelightResult.getPipelineIndex() != limelightPipeline
                && now - lastPipelineSwitchMs > 1000){
            lastPipelineSwitchMs = now;
            limelight3A.pipelineSwitch(limelightPipeline);
        }

        if (limelightResult != null && limelightResult.isValid()){
            double tx = limelightResult.getTx(); // Horizontal  0 is middle
            canSee = true;
            didLastTX = false;
            return tx;
        }else {
            waitTillNotSee = false;
            canSee = false;
            return distanceToHive.GetEstimatedTx();  // Either keep at 0 or add the pinpoint
        }
    }

    /**
     * This is for PID tuning
     * Run this about every loop
     * This will do calculations and move motor to get tx (the amount offset in degrees) to 0
     *
     * @param tx
     * This value is for the amount the turret is off-set in degrees
     * **/
//    public void ChangeAngle(double tx){
//        UpdateMotor(tx);
//    }

    public void ChangeAngleSetPosition(double tx, double rotationV){
        SetTargetPosition (GetTurretCurrentPosition(dcMotorEx) - MiscUtils.getPositionFromDegrees(tx * 1.04 - rotationV * 0.05, 537.7));
    }


    public void SetTargetPosition(double position){
        if (waitTillNotSee || !canSee){
            if (Math.abs(GetTurretCurrentPosition(dcMotorEx) - newPosition) < 400){
                waitTillNotSee = false;
                didLastTX = true;
            }

            if (!canSee && !didLastTX && !waitTillNotSee){
//                didLastTX = true;
//
//                newPosition = GetTurretTargetPosition(dcMotorEx) + lastTx;
//
//                double range = maxPosition - minPosition;
//
//                if (newPosition > maxPosition || newPosition < minPosition){
//                    waitTillNotSee = true;
//                    seeTime = System.currentTimeMillis();
//                    newPosition = minPosition + ((newPosition - minPosition) % range + range) % range;
//                }

//            if (newPosition > maxPosition){
//                newPosition -= positionFromAllAround * (Math.floor((newPosition - maxPosition) / positionFromAllAround) + 1);
//                waitTillNotSee = true;
//            } else if (newPosition < minPosition) {
//                newPosition += positionFromAllAround * (Math.floor((newPosition - minPosition) / positionFromAllAround) + 1);
//                waitTillNotSee = true;
//            }

                SetTurretTargetPosition((int) newPosition, dcMotorEx);
            }
        }else {
//            double positionFromAllAround = MiscUtils.getPositionFromDegrees(360, 537.7);
            newPosition = position;
            previousHeading = positionGetter.r();

            double range = maxPosition - minPosition;

            if (newPosition > maxPosition || newPosition < minPosition){
                waitTillNotSee = true;
                seeTime = System.currentTimeMillis();
                newPosition = minPosition + ((newPosition - minPosition) % range + range) % range;
            }

//            if (newPosition > maxPosition){
//                newPosition -= positionFromAllAround * (Math.floor((newPosition - maxPosition) / positionFromAllAround) + 1);
//                waitTillNotSee = true;
//            } else if (newPosition < minPosition) {
//                newPosition += positionFromAllAround * (Math.floor((newPosition - minPosition) / positionFromAllAround) + 1);
//                waitTillNotSee = true;
//            }

            SetTurretTargetPosition((int) newPosition, dcMotorEx);
        }
    }

    private void SetTargetPositionOverrideAllAround(double position){
        newPosition = position;

        double range = maxPosition - minPosition;

        if (newPosition > maxPosition || newPosition < minPosition){
            waitTillNotSee = true;
            seeTime = System.currentTimeMillis();
            newPosition = minPosition + ((newPosition - minPosition) % range + range) % range;

            previousHeading = positionGetter.r();
        }

        SetTurretTargetPosition((int) newPosition, dcMotorEx);
    }

    private void SetPositionBasedOnPosition(double target){
        SetTurretTargetPosition((int) target, dcMotorEx);
    }

    public void Reset(){
        dtTimer.reset();
        lastTx = 0;

    }

    public void CheckPositionAndSetPower(Telemetry telemetry){
        if (GetTurretTargetPosition(dcMotorEx) > maxPosition || GetTurretTargetPosition(dcMotorEx) < minPosition){
            dcMotorEx.setPower(0);
            if (telemetry != null){
                telemetry.addLine("WENT OVER!");
            }

        }else {

            if (waitTillNotSee || !canSee) { // Rotate the 180 a little slower
                dcMotorEx.setPower(0.5);

                double timeDiff = (System.currentTimeMillis() - seeTime) / 1000.0;

                double currentHeading = positionGetter.r();

                seeTime = System.currentTimeMillis();

                double headingDiff = currentHeading - previousHeading;

                double positionDiff = MiscUtils.getPositionFromDegrees(headingDiff, 537.7);

                double yPosition = positionGetter.y();

                double targetPosition = GetTurretTargetPosition(dcMotorEx) - positionDiff;

                if (rightSide && yPosition > 64){
                    rightSide = false;

                    targetPosition += MiscUtils.getPositionFromDegrees(180, 537.7);
                } else if (!rightSide && yPosition < 64) {
                    rightSide = true;

                    targetPosition += MiscUtils.getPositionFromDegrees(180, 537.7);
                }


                SetTargetPositionOverrideAllAround(targetPosition);

                previousHeading = currentHeading;
            }else{
                dcMotorEx.setPower(Math.max(0.7, Math.abs(GetTurretTargetPosition(dcMotorEx) - GetTurretCurrentPosition(dcMotorEx)) / 1000));  // LLA max .69  min .43
            }
        }
    }

    private double GetTurretCurrentPosition(DcMotorEx motor){

        if (motor == null){
            return 0;
        }


        return motor.getCurrentPosition() + offset;
    }

    private double GetTurretTargetPosition(DcMotorEx motor){

        if (motor == null){
            return 0;
        }

        return motor.getTargetPosition() + offset;
    }

    private void SetTurretTargetPosition(int targetPosition, DcMotorEx motor){

        if (motor == null){
            return;
        }

        motor.setTargetPosition((int) (targetPosition - offset));
    }

    /**
     * RUNS EVERYTHING
     * **/
    public void DoTurret(Limelight3A limelight, Telemetry telemetry){
        // This will turn the turret to the april tags (if it sees it)

        double tx = UpdateTxValue(limelight);

        ChangeAngleSetPosition(tx, positionGetter.getVr());

        double positionChange = MiscUtils.getPositionFromDegrees(tx, 537.7);

        telemetry.addLine("tx: " + tx);
        telemetry.addLine("Amount of Position changes: " + positionChange);

        CheckPositionAndSetPower(telemetry);
    }

    /**
     * RUNS EVERYTHING - NO TELEMETRY
     * **/
    public void DoTurret(Limelight3A limelight){
        // This will turn the turret to the april tags (if it sees it)

        double tx = UpdateTxValue(limelight);

        ChangeAngleSetPosition(tx, positionGetter.getVr());

        double positionChange = MiscUtils.getPositionFromDegrees(tx, 537.7);

        CheckPositionAndSetPower(null);
    }


    //    private void UpdateMotor(double tx){
//        double dt = dtTimer.seconds();
//        dtTimer.reset();
//
//        double targetVelocity = 0;
//        if (dt > 0){
//            targetVelocity = (tx - lastTx) / dt;
//        }
//
//        integralSum += tx * dt;
//
//        integralSum = Math.max(-0.2 / ki, Math.min(-0.2 / ki, integralSum));
//
//        double pTerm = kp * tx;
//        double iTerm = ki * integralSum;
//        double dTerm = kd * targetVelocity;
//
//        double motorPower = pTerm + iTerm + dTerm;
//
//
//        double kf = 0.03;
//        if (Math.abs(tx) > 0.5){
//            motorPower += Math.signum(tx) * kf;
//        }
//
//        motorPower = MiscUtils.Clamp(motorPower, -0.05, 0.05);
//
//        dcMotorEx.setPower(motorPower * -1);
//
//        lastTx = tx;
//    }
}
