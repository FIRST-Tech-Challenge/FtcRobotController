package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.tree.JCTree;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sensor: Husky", group = "Sensor")
public class husky extends LinearOpMode {
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class,"huskyLens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (huskyLens.knock()){
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else{
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if (!rateLimit.hasExpired()){
                continue;
            }
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++){
            telemetry.addData("Block", blocks[1].toString());
        }
        telemetry.update();
    }
}