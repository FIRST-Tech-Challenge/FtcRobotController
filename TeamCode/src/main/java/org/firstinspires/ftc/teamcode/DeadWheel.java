package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Dead wheel testing")
public class DeadWheel extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftDeadWheel = null;
    private DcMotorEx rightDeadWheel = null;
    private DcMotorEx strafeDeadWheel = null;

    @Override
    public void runOpMode() {
        leftDeadWheel = hardwareMap.get(DcMotorEx.class, "left_dead_wheel");
        rightDeadWheel = hardwareMap.get(DcMotorEx.class, "left_dead_wheel");
        strafeDeadWheel = hardwareMap.get(DcMotorEx.class, "left_dead_wheel");

        telemetry.addData("Status", "Initilized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Left dead wheel position", leftDeadWheel.getCurrentPosition());
            telemetry.addData("Right dead wheel position", rightDeadWheel.getCurrentPosition());
            telemetry.addData("Strafe dead wheel position", strafeDeadWheel.getCurrentPosition());
            telemetry.update();
        }
    }
}
