package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class IntakeTest extends LinearOpMode {

    RobotMain bart;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bart = new RobotMain(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                bart.intake.setIntakeMotorPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                bart.intake.setIntakeMotorPower(-gamepad1.left_trigger);
            } else {
                bart.intake.setIntakeMotorPower(0);
            }

            //bart.intake.setHorizontalSlidePower(-gamepad1.right_stick_y*0.5);
            if (gamepad1.a) {
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
            } else if (gamepad1.b) {
                bart.intake.setHorizontalSlideToSavedPosition("max");
            } else {
                bart.intake.setHorizontalSlidePower(-gamepad1.left_stick_y*0.5);
            }

            if (gamepad1.a) {
                bart.intake.openGate();
            } else {
                bart.intake.closeGate();
            }
            telemetry.addData("intake power", bart.intake.intakeMotor.getPower());
            telemetry.addData("intakeAmps", bart.intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("horiz power", bart.intake.horizontalSlide.getPower());
            telemetry.addData("horiz amps", bart.intake.horizontalSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("horiz pos ticks", bart.intake.horizontalSlide.getCurrentPosition());
            telemetry.addData("horiz pos inches", bart.intake.currentInches());
            telemetry.addData("isAtMax", bart.intake.isAtSavedPosition("max"));
            telemetry.addData("gate pos", bart.intake.gate.getPosition());
            telemetry.addData("isOpen", bart.intake.isGateOpen());
            telemetry.update();

        }
    }
}