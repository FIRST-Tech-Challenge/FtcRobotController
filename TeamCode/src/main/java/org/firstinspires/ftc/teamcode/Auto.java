package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int ticks = fLeft.getCurrentPosition();
            telemetry.addLine(String.valueOf(ticks));
            telemetry.update();

            fLeft.setPower(0.1);
            bLeft.setPower(0.1);
            fRight.setPower(0.1);
            bRight.setPower(0.1);

            if (ticks >= 383){
                fLeft.setPower(0);
                bLeft.setPower(0);
                fRight.setPower(0);
                bRight.setPower(0);
            }
        }
    }
}
