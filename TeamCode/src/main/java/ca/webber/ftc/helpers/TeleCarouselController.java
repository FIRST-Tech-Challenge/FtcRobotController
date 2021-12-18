package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleCarouselController {

    private DcMotor p_carouselSpinner;

    public TeleCarouselController (DcMotor carouselSpinner) {
        p_carouselSpinner = carouselSpinner;
    }

    public void update (boolean spin) {
        if (spin) {
            p_carouselSpinner.setPower(0.5);
        } else {
            p_carouselSpinner.setPower(0);
        }
    }
}
