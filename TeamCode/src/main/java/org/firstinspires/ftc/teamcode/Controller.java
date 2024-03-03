package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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
        PixelRumble pixelRumble = new PixelRumble(this, hardwareMap);

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
            pixelRumble.Rumble(gamepad1, gamepad2);

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());

//            DigitalChannel leftSwitch = hardwareMap.get(DigitalChannel.class, "leftSwitch");
//            DigitalChannel rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");
//            DigitalChannel led1 = hardwareMap.get(DigitalChannel.class, "led1");
//            DigitalChannel led2 = hardwareMap.get(DigitalChannel.class, "led2");
//            led1.setMode(DigitalChannel.Mode.OUTPUT);
//            led2.setMode(DigitalChannel.Mode.OUTPUT);
//
//            telemetry.addData("left switch", !leftSwitch.getState());
//            telemetry.addData("right switch", !rightSwitch.getState());
//
//            led1.setState(gamepad2.dpad_up);
//            led2.setState(gamepad2.dpad_up);

            for (DebugData data : debugDataList) {
                telemetry.addData(data.caption, data.value);
            }
            debugDataList.clear();

            telemetry.update();
        }
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
