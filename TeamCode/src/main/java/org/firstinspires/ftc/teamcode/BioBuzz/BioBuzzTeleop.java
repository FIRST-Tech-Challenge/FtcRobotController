package org.firstinspires.ftc.teamcode.BioBuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utils.Controls.ControlsTrigger;
import org.firstinspires.ftc.teamcode.Utils.Controls.GamepadButton;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;
import org.firstinspires.ftc.teamcode.Utils.Movement.PosGetters.PositionGetter;
import org.firstinspires.ftc.teamcode.Utils.Movement.TeleopMovementUtil;
import org.firstinspires.ftc.teamcode.Utils.Positioning.DistanceToHive;
import org.firstinspires.ftc.teamcode.Utils.Shooting.AprilTagsTurretTarget;
import org.firstinspires.ftc.teamcode.Utils.Shooting.LimelightAngle;
import org.firstinspires.ftc.teamcode.Utils.Shooting.TurretConfigurator;

@TeleOp
public class BioBuzzTeleop extends LinearOpMode2026 {

    TeleopMovementUtil teleopMovementUtil;
    AprilTagsTurretTarget aprilTagsTurretTarget;

    long startShot;

    boolean shooting = false;

    boolean isIntakeReversed = false;

    DistanceToHive distanceToHive;

    TurretConfigurator turretConfigurator;
    GoBildaPinpointDriver odo;


    @Override
    public void runOpMode(){
        config();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "POC");
        teleopMovementUtil = new TeleopMovementUtil(gamepad1, mot);
        aprilTagsTurretTarget = new AprilTagsTurretTarget(turret, posGet, 0);
        distanceToHive = new DistanceToHive(posGet);
        turretConfigurator = new TurretConfigurator(this);

        ControlsTrigger shootTrigger = new ControlsTrigger(new GamepadButton(gamepad1, GamepadButton.GButton.right_bumper));
        ControlsTrigger intakeTrigger = new ControlsTrigger(new GamepadButton(gamepad1, GamepadButton.GButton.cross));
        ControlsTrigger inverseLiftTrigger = new ControlsTrigger(new GamepadButton(gamepad1, GamepadButton.GButton.left_bumper));
        ControlsTrigger resetPositon = new ControlsTrigger(new GamepadButton(gamepad1, GamepadButton.GButton.left_stick_button));

        startShot = -1;

        waitForStart();

        while (opModeIsActive()){
            try {
                teleopMovementUtil.UpdateDrive();
                telemetry.addLine("--- BioBuzz Teleop ---");
                DoTurret();
                telemetry.addLine("--- Turret Configurator ---");
                turretConfigurator.runTurret(distanceToHive.GetDistance());

                if (gamepad1.right_bumper){
                    startShot = System.currentTimeMillis();
                    lift.setPower(.6);
                }
                if (inverseLiftTrigger.IsButtonPressed()){
                    lift.setPower(-0.2);
                } else if (!shooting) {
                    lift.setPower(0);
                }
                DoShoot();

                if (intakeTrigger.IsTriggered()){
                    if (isIntakeReversed){
                        isIntakeReversed = false;
                        intake.setPower(-INTAKE_POWER);
                    }else {
                        isIntakeReversed = true;
                        intake.setPower(INTAKE_POWER);
                    }
                }

                if (resetPositon.IsTriggered()){
                    odo.resetPosAndIMU();
                    lla.setPosition(.277);
                    wait(500);
                    lla.setPosition(.611);
                }

                lla.setPosition(LimelightAngle.getAngle(distanceToHive.GetDistance()));

            } catch (Exception e) {
                // Show what crashed instead of freezing on the init screen
                telemetry.addData("Loop error", e.toString());
                StackTraceElement[] trace = e.getStackTrace();
                for (int i = 0; i < Math.min(5, trace.length); i++) {
                    telemetry.addLine("  at " + trace[i]);
                }
            }

            telemetry.addData("Distance From Goal: ", distanceToHive.GetDistance());

            telemetry.update();
        }
    }

    public void DoShoot(){
        if (startShot != -1 && System.currentTimeMillis() - startShot < 1500){ // This should be 1500 if button is trigger
            if (!shooting){
                shooting = true;
                lift.setPower(LIFT_POWER);
            }
        }else {
            if (shooting){
                shooting = false;
                lift.setPower(0);
            }
        }
    }


    public void DoTurret(){
        // This will turn the turret to the april tags (if it sees it)

        double tx = aprilTagsTurretTarget.UpdateTxValue(limelight);

        aprilTagsTurretTarget.ChangeAngleSetPosition(tx, posGet.getVr());

        double positionChange = MiscUtils.getPositionFromDegrees(tx, 537.7);

        telemetry.addLine("tx: " + tx);
        telemetry.addLine("Amount of Position changes: " + positionChange);

        aprilTagsTurretTarget.CheckPositionAndSetPower(telemetry);
    }
}
