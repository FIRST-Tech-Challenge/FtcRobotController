package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.ButtonEX;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

@TeleOp(name="Swerve Test", group="Worlds")
public class SwerveTest extends OpMode{

    RobotConfig r;
    private final ElapsedTime runTime = new ElapsedTime();
    SwerveDriveBase swerve;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialising");
        telemetry.update();
        r = RobotConfig.getInstance(this);
        r.initSystems(RobotConstants.configuredSystems.LEFT_MODULE);
        swerve = r.getSubsystem(RobotConstants.configuredSystems.LEFT_MODULE);
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
        r.systemsStartLoopUpdate();
        swerve.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, (1-gamepad1.right_trigger), false);
        r.systemsEndLoopUpdate();
    }

    @Override
    public void stop(){
        r.closeLogs();
    }
}
