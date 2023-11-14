package org.firstinspires.ftc.teamcode.tasks.samples;

import static android.graphics.Insets.add;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.TaskQueue;

import java.util.ArrayList;

public class TestingMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TaskQueue _taskQueue = new TaskQueue();
        Task move = new MoveTask(_taskQueue, "move");
        Task move2 = new MoveTask(_taskQueue, "move2", new ArrayList<Task>(List.of(move)));

        _taskQueue.start();

        while(!_taskQueue.finished)
            ;
        telemetry.addData( "x" ,RobotConfig.getInstance().x);
        telemetry.update();
    }
}
