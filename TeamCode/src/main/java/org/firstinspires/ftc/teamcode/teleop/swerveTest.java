package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamUtil.log;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

@TeleOp(name="Swerve Test", group="Worlds")
public class swerveTest extends OpMode{

    robotConfig r;
    private final ElapsedTime runTime = new ElapsedTime();
    boolean flipInputPressed = false;
    boolean robotFlipped = false;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialising");
        telemetry.update();
        r = new robotConfig(this, robotConstants.configuredSystems.BOTH_MODULES);
        telemetry.addData("Status", "Initialised");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop(){
        if(!flipInputPressed && gamepad1.a){
            robotFlipped = !robotFlipped;
            flipInputPressed = true;
        }
        else if (!gamepad1.a){
            flipInputPressed = false;
        }
        r.encoderRead.encoderBulkRead();
        r.swerve.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, (1-gamepad1.right_trigger), false);

        telemetry.update();
    }

    @Override
    public void stop(){
        robotConfig.closeLogs();
    }
}
