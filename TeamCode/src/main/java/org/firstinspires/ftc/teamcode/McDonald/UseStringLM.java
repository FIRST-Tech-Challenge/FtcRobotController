package org.firstinspires.ftc.teamcode.McDonald;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@TeleOp
public class UseStringLM extends OpMode {
    @Override
    public void init() {
        String myName = "Landen McDonald";

        int grade = 9;
        // Created a new variable called "grade" and set it to "9"
        telemetry.addData("Hello", myName);

        telemetry.addData("grade",grade);

    }

    @Override
    public void loop() {
        int x = 5;
        // x is visible here
        {
            int y = 4;
            // only x is visible here
        }
        // only x is visible here
    }
}
