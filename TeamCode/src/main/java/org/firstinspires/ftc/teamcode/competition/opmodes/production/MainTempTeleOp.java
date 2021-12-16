package org.firstinspires.ftc.teamcode.competition.opmodes.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.competition.scripts.teleop.MainTempTeleOpScript;
import org.firstinspires.ftc.teamcode.competition.utils.gamepads.GamepadManager;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardServo;
import org.firstinspires.ftc.teamcode.competition.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.io.OutputSpace;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorLeftLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorRightLiftMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeLiftingServoLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeSpinningMotorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.TankDrivetrainLocation;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.ControlLoopManager;

@TeleOp(name="MainTempTeleOp", group="production")
public class MainTempTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MainTempTeleOpScript script = new MainTempTeleOpScript(this);
        ControlLoopManager loopManager = new ControlLoopManager(this);
        waitForStart();
        resetStartTime();
        while(loopManager.shouldContinue()) {
            script.main();
        }
        script.stop();
    }

}
