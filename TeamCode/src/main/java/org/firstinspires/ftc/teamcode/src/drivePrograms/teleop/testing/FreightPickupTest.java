package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@Disabled
@TeleOp(name = "Test picking up the freight")
public class FreightPickupTest extends TeleOpTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        DcMotor frontIntake = hardwareMap.dcMotor.get(GenericOpModeTemplate.frontIntakeMotorName);
        DcMotor backIntake = hardwareMap.dcMotor.get(GenericOpModeTemplate.backIntakeMotorName);
        this.initDriveTrain();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            driveTrain.gamepadControl(gamepad1,gamepad2);

            if (gamepad2.a){
                frontIntake.setPower(1);
                backIntake.setPower(1);
            }else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

        }

    }
}
