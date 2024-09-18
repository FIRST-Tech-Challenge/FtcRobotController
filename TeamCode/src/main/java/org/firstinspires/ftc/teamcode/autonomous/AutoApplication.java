package org.firstinspires.ftc.teamcode.autonomous;

public class AutoApplication extends AutoMain{

    //gamepad1 is driver, gamepad2 is arm

    @Override
    public void init() {
        telemetry.addLine("x = Blue, b = RED");
    }

    @Override
    public void loop() {

    }

    @Override
    public void init_loop(){
        if (gamepad1.x) {
            us = Alliance.BLUE;
            others = Alliance.RED;
        }
        else if (gamepad1.b) {
            us = Alliance.RED;
            others = Alliance.BLUE;
        }
        telemetry.update();
    }

    @Override
    public void start(){

    }
}