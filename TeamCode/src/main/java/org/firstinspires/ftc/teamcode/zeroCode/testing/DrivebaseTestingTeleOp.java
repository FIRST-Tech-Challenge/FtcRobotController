package org.firstinspires.ftc.teamcode.zeroCode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.Constants;
import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.FieldData;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Intakes;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Ramp;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Turret;
import org.firstinspires.ftc.teamcode.zeroCode.utils.drive.DriveFC;

public class DrivebaseTestingTeleOp extends OpMode {
    DriveFC drive;
    Intakes intake;
    Ramp ramp;
    Turret turret;

    @Override
    public void init() {
        Constants.hardware.resetHardwareMaps(telemetry);
        Constants.hardware.initializeIntakes = true;
        Constants.hardware.initializeDrive = true;
        Constants.hardware.initializeRamp = true;
        Constants.hardware.initializeIMU = true;
        Constants.hardware.initializeTurret = true;
        Constants.hardware.initializeHardwareMaps(telemetry, hardwareMap);

        drive = new DriveFC();
        intake = new Intakes();
        ramp = new Ramp();
        turret = new Turret();

        telemetry.update();
    }

    @Override
    public void loop() {
        //Drive command:
        double[] motorPowers = drive.calculateDriveMecanumFC(
                FieldData.getHeading(),
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y);

        drive.powerMotors(
                motorPowers[0],
                motorPowers[1],
                motorPowers[2],
                motorPowers[3]
        );

        if (gamepad1.x) {
            intake.powerPollenIntake(1);
        } else {
            intake.powerPollenIntake(0);
        }

        if (gamepad1.y) {
            intake.powerNectarIntake(1);
        } else {
            intake.powerNectarIntake(0);
        }


        if (gamepad1.a) {
            ramp.turnToNectar();
        } else if (gamepad1.b) {
            ramp.turnToNectar();
        }

        turret.powerTurret(
                gamepad1.left_trigger > gamepad1.right_trigger ? 
                        gamepad1.left_trigger : -gamepad1.right_trigger
        );

        telemetry.update();
    }
}
