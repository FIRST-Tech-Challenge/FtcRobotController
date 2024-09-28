package org.firstinspires.ftc.masters.components;

public class ControllerMap {
    public enum Buttons {
        A,
        B,
        X,
        Y,


    }

    public enum Easing {
        QUAD_IN,
        QUAD_OUT,
        QUAD_IN_OUT,
        CUBE,
        QUARTs
    }

    private class KeyBind {
        public KeyBind() {

        }
    }

    public KeyBind[] bindings;
    public ControllerMap() {

    }
}
