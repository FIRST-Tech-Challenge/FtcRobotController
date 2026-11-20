package org.firstinspires.ftc.teamcode.BioBuzz;

import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;
import org.firstinspires.ftc.teamcode.Utils.Shooting.Turret;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode2026 {

    static final double INTAKE_POWER = 0.8;
    static final double LIFT_POWER = 1.0;

    static final double TICKS_PER_REV = 28;

    Turret turretAimer;

    boolean isIntakeOn = false;
    boolean isLiftOn = false;



    @Override
    public void runOpMode() throws InterruptedException {
        config();

        turretAimer = new Turret(turret, posGet, MiscUtils.getMotorPositionFromDegrees(90, 537.7));

        waitForStart();




        while (opModeIsActive()){
            DoLoop(); // Do this every frame during auto

            if (gamepad1.cross){
                ShootAll();
            }
        }
    }

    public void ShootAll(){
        intake.setPower(INTAKE_POWER);
        //Lift only runs while left bumper is held
        lift.setPower(LIFT_POWER);

        sleep(2000);

        flywheel.setVelocity(0);
        intake.setPower(0);
        //Lift only runs while left bumper is held
        lift.setPower(0);
    }

    public void DoLoop(){
        DoTurret();

        telemetry.update();
    }

    public void DoTurret(){
        // This will turn the turret to the april tags (if it sees it)

        double tx = turretAimer.UpdateTxValue(limelight);

        turretAimer.ChangeAngleSetPosition(tx, posGet.getVr());

        double positionChange = MiscUtils.getMotorPositionFromDegrees(tx, 537.7);

        telemetry.addLine("tx: " + tx);
        telemetry.addLine("Amount of Position changes: " + positionChange);

        turretAimer.CheckPositionAndSetPower(telemetry);
    }
}
