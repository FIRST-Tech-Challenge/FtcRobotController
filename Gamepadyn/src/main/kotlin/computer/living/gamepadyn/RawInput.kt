package computer.living.gamepadyn

import computer.living.gamepadyn.InputType.*;

enum class RawInput(val type: InputType, val axes: Int = 0) {
    FACE_DOWN           (DIGITAL), // Generic face button (= A)
    FACE_RIGHT          (DIGITAL), // Generic face button (= B)
    FACE_LEFT           (DIGITAL), // Generic face button (= X)
    FACE_UP             (DIGITAL), // Generic face button (= Y)

    FACE_A              (DIGITAL), // XB face button (= DOWN)
    FACE_B              (DIGITAL), // XB face button (= RIGHT)
    FACE_X              (DIGITAL), // XB face button (= LEFT)
    FACE_Y              (DIGITAL), // XB face button (= UP)

    FACE_CROSS          (DIGITAL), // PS face button (= A)
    FACE_CIRCLE         (DIGITAL), // PS face button (= B)
    FACE_SQUARE         (DIGITAL), // PS face button (= X)
    FACE_TRIANGLE       (DIGITAL), // PS face button (= Y)

    BUMPER_LEFT         (DIGITAL),
    BUMPER_RIGHT        (DIGITAL),

    DPAD_UP             (DIGITAL),
    DPAD_DOWN           (DIGITAL),
    DPAD_LEFT           (DIGITAL),
    DPAD_RIGHT          (DIGITAL),

    STICK_LEFT_BUTTON   (DIGITAL),
    STICK_RIGHT_BUTTON  (DIGITAL),
    STICK_LEFT          (ANALOG, 2),
    STICK_RIGHT         (ANALOG, 2),

    TRIGGER_LEFT        (ANALOG, 1),
    TRIGGER_RIGHT       (ANALOG, 1),

//    SPECIAL_GUIDE(DIGITAL),
//    SPECIAL_START(DIGITAL),
//    SPECIAL_BACK(DIGITAL),
//    SPECIAL_SHARE(DIGITAL),
//    SPECIAL_OPTIONS(DIGITAL),
    ;

//    constructor() : this(ActionType.DIGITAL, 0);
}
