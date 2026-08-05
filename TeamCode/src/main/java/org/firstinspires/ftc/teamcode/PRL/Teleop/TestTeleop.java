package org.firstinspires.ftc.teamcode.PRL.Teleop;

//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ActionClass;
import org.firstinspires.ftc.teamcode.PRL.Class.TurretShooter;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.TurretClass;

//@Configurable
@TeleOp
public class TestTeleop extends LinearOpMode {
    LimelightClass limelight;
    TurretClass turret;
    public static double kP = 0.015;
    public static double kD = 0.003;
    public static int TARGET_TAG_ID = 1;
    double lastError = 0;

    ActionClass action;
    TurretShooter ts;

    @Override
    public void runOpMode() {
        limelight = new LimelightClass(hardwareMap);
        turret = new TurretClass(hardwareMap);
        action = new ActionClass(hardwareMap);
        ts = new TurretShooter(hardwareMap);

        limelight.setTargetTagID(TARGET_TAG_ID);
        limelight.start();

        waitForStart();
        while (opModeIsActive()) {
            limelight.update();
            intake();
            shooter();
            if (limelight.hasTarget()) {
                double error = limelight.getTx();
                double derivative = error - lastError;
                lastError = error;
                double power = kP * error + kD * derivative;
                power = Math.max(-0.6, Math.min(0.6, power));
                turret.setPower(power);

                telemetry.addData("tx", error);
                telemetry.addData("Power", power);
            } else {
                turret.setPower(0);
                telemetry.addLine("No Target");
            }
            telemetry.update();
        }

        limelight.stop();
    }

    void intake(){
        if (gamepad1.a){
            action.Intake_On();
        }else if (gamepad1.b){
            action.Intake_R();
        }else{
            action.Intake_Off();
        }
    }

    void shooter(){
        if (gamepad1.dpad_down){
            ts.preheat();
        }else if (gamepad1.dpad_up){
            ts.shoot();
        }
    }
}
