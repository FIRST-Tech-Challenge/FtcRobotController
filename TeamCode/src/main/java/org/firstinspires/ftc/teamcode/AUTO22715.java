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
        // Put initialization blocks here.
        huskyLens.init(hardwareMap, telemetry);

        // 创建一个新的Thread去检测摄像头，并且显示当前tag所放置的方向
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
                        sleep(500);
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

        waitForStart();

        bQuit = true;
        //
        //等待检测方向的子线程退出
        sleep(1000);

        while (opModeIsActive()) {
            // Put run blocks here.
            do{
                //
                //更新一次方向
                huskyLens.calculateDirection();
                direction = huskyLens.getTagDirection();
                telemetry.addData(">>>", direction);
                telemetry.update();
            } while(direction != SensorHuskyLens.TagDirection.UNKOWN);

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