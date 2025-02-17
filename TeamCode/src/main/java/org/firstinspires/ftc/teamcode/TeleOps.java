package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GyroBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.AutomationBot;
import org.firstinspires.ftc.teamcode.bots.LimelightBot;
import org.firstinspires.ftc.teamcode.bots.PivotBot;
import org.firstinspires.ftc.teamcode.bots.PinchBot;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;
import org.firstinspires.ftc.teamcode.sample.Sample;

@TeleOp(name = "Drive")
public class TeleOps extends LinearOpMode {
    private RollerIntakeBot robot = new RollerIntakeBot(this);

    private Sample lastSample = null;

    private boolean blueAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = false;

        robot.init(hardwareMap);

        telemetry.addData("blue alliance:", blueAlliance);

        while (opModeInInit()) {
            if (gamepad1.dpad_right) {
                blueAlliance = !blueAlliance;
                telemetry.addData("blue alliance:", blueAlliance);
            }
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()){

            telemetry.setMsTransmissionInterval(11);

            telemetry.addData("slide position", robot.getSlidePosition());
            telemetry.addData("pivot position", robot.getPivotPosition());
            telemetry.addData("vR",robot.rightFront.getCurrentPosition() );
            telemetry.addData("vL", robot.rightRear.getCurrentPosition());
            telemetry.addData("h", robot.leftRear.getCurrentPosition());

            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }

        }
        robot.close();
    }

}
