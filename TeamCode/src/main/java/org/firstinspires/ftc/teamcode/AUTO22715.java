package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AUTO22715 (Blocks to Java)")

public class AUTO22715 extends LinearOpMode {

    SensorHuskyLens huskyLens = new SensorHuskyLens();
    private final int READ_PERIOD = 1;

    SensorHuskyLens.TagDirection direction = SensorHuskyLens.TagDirection.UNKOWN;

    AUTOBLleft left = new AUTOBLleft() ;
    AUTOBLmid middle = new AUTOBLmid();
    AUTOBLright right = new AUTOBLright();
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    Boolean bQuit = false;
    @Override
    public void runOpMode() {
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        // Put initialization blocks here.
        huskyLens.init(hardwareMap, telemetry);
        huskyLens.initBeforeOpMode();

        // 创建一个新的Thread对象
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!bQuit){
                    // 这是子线程要执行的代码
                    try {
                        huskyLens.calculateDirection();
                        direction = huskyLens.getTagDirection();
                        telemetry.addData(">>>", direction);
                        telemetry.update();
                        sleep(1000);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        // 启动子线程
        bQuit = false;
        thread.start();


        left.init(this);
        middle.init(this);
        right.init(this);

        left.initBeforeOpMode();
        middle.initBeforeOpMode();
        right.initBeforeOpMode();

        waitForStart();

        bQuit = true;
        sleep(10);

        telemetry.addData(">>>", "init complete");
        telemetry.update();
        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            // Put run blocks here.
            if(direction == SensorHuskyLens.TagDirection.UNKOWN){
                huskyLens.calculateDirection();
                direction = huskyLens.getTagDirection();
                telemetry.addData(">>>", direction);
                telemetry.update();
            }

            if(direction == SensorHuskyLens.TagDirection.LEFT){
                left.runOpMode();
                break;
            } else if(direction == SensorHuskyLens.TagDirection.MIDDLE){
                middle.runOpMode();
                break;
            } else if(direction == SensorHuskyLens.TagDirection.RIGHT){
                right.runOpMode();
                break;
            }
        }
    }
}