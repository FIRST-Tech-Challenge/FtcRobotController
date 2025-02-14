package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class HangOnly extends OpMode {
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public enum SlidesState {
        DOWN,
        UP,
        HANG
    }
    public SlidesState state;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryAxonV1 axon = new DeliveryAxonV1();
    @Override
    public void init() {
        slides.init(hardwareMap);
        axon.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(state == SlidesState.DOWN){
            if (slides.getLeftSlidePosition() != 0 || slides.getRightSlidePosition() != 0) {
                slides.runLeftSlideToPosition(0, 0.9);
                slides.runRightSlideToPosition(0, 0.9);
            } else if (slides.getRightSlidePosition() != 0 && slides.getLeftSlidePosition() != 0) {
                slides.runLeftSlideToPosition(0, 0.9);
                slides.runRightSlideToPosition(0, 0.9);
            }
            if(gamepad1.x && buttonTimer.seconds() > 0.3){
                state = SlidesState.UP;
                buttonTimer.reset();
            }
            axon.setPosition(constants.DELIVERY_LEVEL_TWO_ASCENT);
        }
        else if(state == SlidesState.UP){
            slides.runLeftSlideToPosition(constants.LEFT_SLIDE_LEVEL_TWO_ASCENT_HOOK, 0.9);
            slides.runRightSlideToPosition(constants.RIGHT_SLIDE_LEVEL_TWO_ASCENT_HOOK, 0.9);
            if(gamepad1.x && buttonTimer.seconds() > 0.3){
                state = SlidesState.HANG;
                buttonTimer.reset();
            }
        }
        else if(state == SlidesState.HANG) {
            slides.runLeftSlideToPosition(0, 1);
            slides.runRightSlideToPosition(0, 1);
        }
    }
}
