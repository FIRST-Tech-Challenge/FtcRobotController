/*package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
@TeleOp
public class TELEOPtest3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor
    private Servo leftGripper;
    private Servo rightGripper;
    private Servo launcherServo = null;
    private Servo DroneCoverServo = null;
    private DcMotor armMotor = null;
    MotionHardware robot = new MotionHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();
    public static void main(String[] args) {
        // Creating two instances of the Runnable implementation
        RunnableTask task1 = new RunnableTask("Driver 1");
        RunnableTask task2 = new RunnableTask("Driver 2");

        // Creating two Thread objects and passing the Runnable instances
        Thread thread1 = new Thread(task1);
        Thread thread2 = new Thread(task2);

        // Starting the threads
        thread1.start();
        thread2.start();
    }
}

// Runnable implementation that contains the task to be performed by the threads
class RunnableTask implements Runnable {
    private String threadName;

    public RunnableTask(String threadName) {
        this.threadName = threadName;
    }

    @Override
    public void run() {
        for (int i = 1; i <= 5; i++) {
            System.out.println(threadName + " - Count: " + i);


        }
    }
}*/