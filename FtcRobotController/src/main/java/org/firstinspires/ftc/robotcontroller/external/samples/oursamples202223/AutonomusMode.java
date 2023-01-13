package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomus(name = "Chance-testing")
// Documentation : https://docs.revrobotics.com/kickoff-concepts/freight-frenzy-2021-2022/autonomous
public class AutoMode extends LinearOpMode {
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;


    // TODO -- work on logic
    public void RightTurn() {
        System.out.println("Right Turn");
    }

    public void LeftTurn() {
        System.out.println("Left turn");
    }

    public void Forward() {
        System.out.println("move forward");
    }

    public void Reverse() {
        System.out.println("move backwards");
    }

    // TODO: add in arm and other features


    public void runOpMode() {
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

        ElapsedTime runtime = new ElapsedTime();

        int AUTONOMUS_MODE_SECONDS = 30


        if (opModeIsActive()) {

            // resetting the time to 0
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < AUTONOMUS_MODE_SECONDS + 1) {
                // Entering while loop

                // Arbitray
                if (runtime.seconds() == 3) {
                    System.out.println("We have ran for 3 seconds - should probably move");
                    RightTurn();
                }

                if (runtime.seconds() == 6) {
                    System.out.println("six seconds in");
                    Forward();
                }

                // continue logic
            }
        }

    }
}
