package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "util: Vertical Slide", group = "Utilities")
public class VerticalSlideTool extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware h = new Hardware(hardwareMap);
        DcMotor it = h.verticalSlide;
//        it.setPower(0.0);
//        it.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        it.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
//        h.verticalLift.setTargetPosition(500);
        while (opModeIsActive()) {
            telemetry.addData("position", it.getCurrentPosition());
            telemetry.addData("directed power", h.verticalLift.getPower());
            telemetry.addData("target", h.verticalLift.getTargetPosition());
            telemetry.update();
            if (gamepad1.y) h.verticalLift.setTargetPosition(Hardware.VLIFT_SCORE_HIGH);
            else if (gamepad1.b) h.verticalLift.setTargetPosition(300);
            else if (gamepad1.a) h.verticalLift.setTargetPosition(0);
//            else h.verticalLift.setPower(0);
            h.verticalLift.update();
        }
    }
}
