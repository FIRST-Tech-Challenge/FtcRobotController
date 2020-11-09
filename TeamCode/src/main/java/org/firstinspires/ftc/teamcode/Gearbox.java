package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.Servo;

public class Gearbox extends LinearOpMode {

    private Servo left = null;
    private double lmax = .01; // Maximum rotational position
    private double lmin = .99; // Minimum rotational position


    private String currentPos = "up";

    public Gearbox (Servo l) {
        left = l;
    }

    public void grab() {
        left.setPosition(lmin);
    }
    public void release() {
        left.setPosition(lmax);
    }

    public void nextPos() {
        if(currentPos == "closed") {
            currentPos = "open";
            release();
        } else if(currentPos == "open") {
            currentPos = "closed";
            grab();
        }
    }

    public String getPos () {
        return currentPos;
    }

    public void runOpMode() {

    }
}