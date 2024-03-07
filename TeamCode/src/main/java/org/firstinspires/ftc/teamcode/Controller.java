package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "controller movement", group = "SA_FTC")
public class Controller extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    List<DebugData> debugDataList = new ArrayList<DebugData>();

    @Override
    public void runOpMode() {
        MovementUtils movementUtils = new MovementUtils(this, hardwareMap);
        ArmUtils armUtils = new ArmUtils(this, hardwareMap);
        DetectPixel detectPixel = new DetectPixel(this, hardwareMap);
        DrivingAssist drivingAssist = new DrivingAssist(this, hardwareMap, detectPixel);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        armUtils.startupSequence();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //movementUtils.movement(gamepad1);
            movementUtils.vectorsMovement(gamepad1);
            armUtils.roller(gamepad2);
            armUtils.extend(gamepad2);
            armUtils.lift(gamepad2);
            armUtils.grip(gamepad2);
            armUtils.runSequences(gamepad2);
            //drivingAssist.rumble(gamepad1, gamepad2);
            drivingAssist.gripLed(gamepad1, gamepad2);

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            for (DebugData data : debugDataList) {
                telemetry.addData(data.caption, data.value);
            }
            debugDataList.clear();

            telemetry.update();
        }

        detectPixel.closeTfod();
    }

    public void Debug(String caption, Object value) {
        DebugData newData = new DebugData(caption, value);
        debugDataList.add(newData);
    }
}

class DebugData {
    public String caption;
    public Object value;

    public DebugData(String caption, Object value) {
        this.caption = caption;
        this.value = value;
    }
}
