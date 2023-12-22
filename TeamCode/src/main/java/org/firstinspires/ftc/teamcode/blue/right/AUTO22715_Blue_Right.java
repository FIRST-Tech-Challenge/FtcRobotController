package org.firstinspires.ftc.teamcode.blue.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blue.SensorHuskyLens_Blue;

@Autonomous(name = "AUTO22715_Blue_Right (Blue Right)")
public class AUTO22715_Blue_Right extends LinearOpMode {
    SensorHuskyLens_Blue huskyLens = new SensorHuskyLens_Blue();
    SensorHuskyLens_Blue.TagDirection direction = SensorHuskyLens_Blue.TagDirection.UNKOWN;

    AUTOBLleft_Blue_Right left = new AUTOBLleft_Blue_Right() ;
    AUTOBLmid_Blue_Right middle = new AUTOBLmid_Blue_Right();
    AUTOBLright_Blue_Right right = new AUTOBLright_Blue_Right();
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

        waitForStart();

        bQuit = true;
        //
        //等待检测方向的子线程退出
        sleep(1500);

        while (opModeIsActive()) {
            // Put run blocks here.
            do{
                //
                //更新一次方向
                huskyLens.calculateDirection();
                direction = huskyLens.getTagDirection();
                telemetry.addData(">>>", direction);
                telemetry.update();
            } while(direction == SensorHuskyLens_Blue.TagDirection.UNKOWN);


            if(direction == SensorHuskyLens_Blue.TagDirection.LEFT){
                left.runOpMode();
                break;
            } else if(direction == SensorHuskyLens_Blue.TagDirection.MIDDLE){
                middle.runOpMode();
                break;
            } else if(direction == SensorHuskyLens_Blue.TagDirection.RIGHT){
                right.runOpMode();
                break;
            }
        }
    }
}