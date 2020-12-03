package com.team16488;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.team16488.common.RobotMap;
import com.team16488.compoonents.MecanumDrive;

/**
 * This is the main OpMode for the driver operated portion of the event
 * HERE WE RUN THE METHODS ASSOCIATED WITH THE COMPONENTS OF THE ROBOT
 */
public class robot extends OpMode {
    public RobotMap robotMap = new RobotMap();
    public MecanumDrive drive;

    @Override
    public void init() {
        this.robotMap.mapHardware(hardwareMap);
        this.drive = new MecanumDrive(this.robotMap);
    }

    @Override
    public void loop() {
        // operator drive
        drive.operatorMecanumDrive(gamepad1.left_stick_x,
                                   gamepad1.left_stick_y,
                                   gamepad1.right_stick_x);

    }
}
