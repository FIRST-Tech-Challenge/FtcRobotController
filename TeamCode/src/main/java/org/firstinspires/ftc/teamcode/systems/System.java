package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controller;

public abstract class System {

    protected HardwareMap hw;
    protected Controller controller;

    public System(HardwareMap hw, Controller controller){
        this.hw = hw;
        this.controller = controller;
    }

    public abstract void init();
    public abstract void update();

}
