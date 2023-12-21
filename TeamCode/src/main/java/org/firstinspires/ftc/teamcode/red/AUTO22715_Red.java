package org.firstinspires.ftc.teamcode.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blue.AUTOBLleft_Blue;

@Autonomous(name = "AUTO22715_Red (Red)")

public class AUTO22715_Red extends LinearOpMode {
    SensorHuskyLens_Red huskyLens = new SensorHuskyLens_Red();
    SensorHuskyLens_Red.TagDirection direction = SensorHuskyLens_Red.TagDirection.UNKOWN;

    AUTOBLleft_Red left = new AUTOBLleft_Red() ;
    AUTOBLmid_Red middle = new AUTOBLmid_Red();
    AUTOBLright_Red right = new AUTOBLright_Red();
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
            } while(direction != SensorHuskyLens_Red.TagDirection.UNKOWN);

            if(direction == SensorHuskyLens_Red.TagDirection.LEFT){
                left.runOpMode();
                break;
            } else if(direction == SensorHuskyLens_Red.TagDirection.MIDDLE){
                middle.runOpMode();
                break;
            } else if(direction == SensorHuskyLens_Red.TagDirection.RIGHT){
                right.runOpMode();
                break;
            }
        }
    }
}