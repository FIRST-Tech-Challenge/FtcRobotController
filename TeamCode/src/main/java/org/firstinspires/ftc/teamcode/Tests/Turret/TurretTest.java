package org.firstinspires.ftc.teamcode.Tests.Turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Controls.ControlsTrigger;
import org.firstinspires.ftc.teamcode.Utils.Controls.GamepadButton;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;
import org.firstinspires.ftc.teamcode.Utils.Movement.TeleopMovementUtil;
import org.firstinspires.ftc.teamcode.Utils.Shooting.Turret;

@TeleOp
@Disabled
public class TurretTest extends LinearOpMode2026 {

    double num = 0;

    double currentPosition = 0;

    double targetPosition = 0;

    double tolerance = 0.1;

    ControlsTrigger triggerDrivingTracking;

    boolean isDrivingTracking = false;

    @Override
    public void runOpMode(){
        config();

        Turret turret = new Turret(this.turret, posGet, 0);

        triggerDrivingTracking = new ControlsTrigger(new GamepadButton(gamepad1, GamepadButton.GButton.a));

        TeleopMovementUtil teleopMovementUtil = new TeleopMovementUtil(gamepad1, mot);

        waitForStart();
        turret.Reset(); // This isn't really needed any more unless if we do PID tuning for this

        while (opModeIsActive()){
            boolean trig = triggerDrivingTracking.IsTriggered();

            teleopMovementUtil.UpdateDrive();

            if (trig){
                isDrivingTracking = !isDrivingTracking;
            }

            if (isDrivingTracking){
                double tx = turret.UpdateTxValue(limelight);

                turret.ChangeAngleSetPosition(tx, posGet.getVr());
                turret.CheckPositionAndSetPower(telemetry);

                double positionChange = MiscUtils.getMotorPositionFromDegrees(tx, 537.7);

                telemetry.addLine("tx: " + tx);
                telemetry.addLine("Amount of Position changes: " + positionChange);
            }else {
                //teleopMovementUtil.DeadDrive();
            }

            if (this.turret.getTargetPosition() > turret.maxPosition || this.turret.getTargetPosition() < turret.minPosition){
                this.turret.setPower(0);
                telemetry.addLine("WENT OVER!");
            }else {
                this.turret.setPower(0.7);  // LLA max .69  min .43
            }

            if (gamepad1.dpad_up){
                targetPosition += 10;
                turret.SetTargetPosition(targetPosition);
                sleep(100);
            }

            if (gamepad1.dpad_down){
                targetPosition -= 10;
                turret.SetTargetPosition(targetPosition);
                sleep(100);
            }

            if (gamepad1.x){
                turret.Reset();

                this.turret.setPower(0.05);

                turret.ChangeAngleSetPosition(targetPosition, posGet.getVr());
            }else {
                targetPosition = this.turret.getTargetPosition();
            }

            telemetry.addLine("CURRENTLY DOING: " + (isDrivingTracking ? "Driving Tracking" : "NONE"));
            telemetry.addData("Rotation V", posGet.getVr());
            telemetry.addData("Current Position", this.turret.getCurrentPosition());
            telemetry.addData("Text TX", num);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Max", turret.maxPosition);
            telemetry.addData("Min", turret.minPosition);
            telemetry.addLine("Controls: \nA: Run test\nDPad up and down: Changes test rotation amount\nB: Run april tag");
            telemetry.update();
        }
    }

    //                num = aprilTagsTurretTarget.UpdateTxValue(limelight);

    // aprilTagsTurretTarget.ChangeAngleSetPosition(turret.getCurrentPosition());


//                aprilTagsTurretTarget.Reset();
//
//                double targetPosition = currentPosition + num;
//
//                while (Math.abs(num) > tolerance){
//                    SetCurrentPosition();
//                    num = targetPosition - currentPosition;
//                    aprilTagsTurretTarget.ChangeAngle(num);
//                }
//
//                turret.setPower(0);

    void SetCurrentPosition(){
        currentPosition = MiscUtils.getDegreesFromMotor(turret.getCurrentPosition(), 537.7) * 14 / 43;
    }

}
