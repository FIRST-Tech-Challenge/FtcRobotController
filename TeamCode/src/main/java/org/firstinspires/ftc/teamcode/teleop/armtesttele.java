package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.Hardware2;
@Disabled
@TeleOp(name="armtesttele", group="Linear Opmode")
public class armtesttele extends BaseOpMode {
    private final Hardware2 robot = new Hardware2(false);
    @Override
    public Hardware2 getRobot() {
        return null;
    }

    @Override
    public void runOpMode() throws InterruptedException {
    robot.initTeleOpIMU(hardwareMap);
    waitForStart();
    robot.getArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ElapsedTime time = new ElapsedTime();
    while(opModeIsActive()){
//        telemetry.addLine(String.valueOf(robot.getArm().getCurrentPosition()));
//        sleep(30000);
        telemetry.addLine(String.valueOf(gamepad2.left_trigger));
        telemetry.update();
        sleep(1000);
    }
    }
}
