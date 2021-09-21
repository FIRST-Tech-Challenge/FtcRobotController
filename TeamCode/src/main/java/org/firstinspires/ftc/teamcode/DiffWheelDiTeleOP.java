package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class DiffWheelDiTeleOP extends OpMode {
    Hardware robot = new Hardware();
    final double countPerRev = 384.5;
    final double wheelDi0 = 4.75;
    final double countPerIN0 = countPerRev/(wheelDi0*Math.PI);
    final double wheelDi1 = 3.75;
    final double countPerIN1 = countPerRev/(wheelDi1*Math.PI);
    final double wheelDi2 = 4.75;
    final double countPerIN2 = countPerRev/(wheelDi2*Math.PI);
    final double wheelDi3 = 3.75;
    final double countPerIN3 = countPerRev/(wheelDi3*Math.PI);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y*.1;
        double turn = gamepad1.right_stick_x*.5;

        int drive0 = (int)(drive * countPerIN0);
        int drive1 = (int)(drive * countPerIN1);
        int drive2 = (int)(drive * countPerIN2);
        int drive3 = (int)(drive * countPerIN3);

        if (turn < 0) {
            robot.m0.setPower(drive0);
            robot.m1.setPower(drive1+turn);
            robot.m2.setPower(drive2);
            robot.m3.setPower(drive3+turn);
        }
        else{
            robot.m0.setPower(drive0-turn);
            robot.m1.setPower(drive1);
            robot.m2.setPower(drive2-turn);
            robot.m3.setPower(drive3);
        }
    }
}
