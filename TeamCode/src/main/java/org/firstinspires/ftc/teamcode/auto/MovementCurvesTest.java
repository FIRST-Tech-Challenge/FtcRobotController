package org.firstinspires.ftc.teamcode.auto;

public class MovementCurvesTest {
    public static void main(String[] args) {

        assert (MovementCurves.circleCurve(-.01) == -1);
        assert (MovementCurves.exponentialEaseIn(-.01) == -1);
        assert (MovementCurves.exponentialEaseOut(-.01) == -1);
        assert (MovementCurves.parametricCurve(-.01) == -1);
        assert (MovementCurves.quadraticCurve(-.01) == -1);
        assert (MovementCurves.normalCurve(-.01) == -1);
        assert (MovementCurves.roundedSquareCurve(-.01) == -1);
        assert (MovementCurves.sinCurve(-.01) == -1);
        assert (MovementCurves.linear(-.01) == -1);

        assert (MovementCurves.circleCurve(1.01) == -1);
        assert (MovementCurves.exponentialEaseIn(1.01) == -1);
        assert (MovementCurves.exponentialEaseOut(1.01) == -1);
        assert (MovementCurves.parametricCurve(1.01) == -1);
        assert (MovementCurves.quadraticCurve(1.01) == -1);
        assert (MovementCurves.normalCurve(1.01) == -1);
        assert (MovementCurves.roundedSquareCurve(-.01) == -1);
        assert (MovementCurves.sinCurve(1.01) == -1);
        assert (MovementCurves.linear(1.01) == -1);

        System.out.println("inRange checks cleared");

        for (double i = 0; i <= 1; i += .01) {
            assert (MovementCurves.circleCurve(i) >= 0 && MovementCurves.circleCurve(i) <= 1);
            assert (MovementCurves.exponentialEaseIn(i) >= 0 && MovementCurves.exponentialEaseIn(i) <= 1);
            assert (MovementCurves.exponentialEaseOut(i) >= 0 && MovementCurves.exponentialEaseOut(i) <= 1);
            assert (MovementCurves.parametricCurve(i) >= 0 && MovementCurves.parametricCurve(i) <= 1);
            assert (MovementCurves.quadraticCurve(i) >= 0 && MovementCurves.quadraticCurve(i) <= 1);
            assert (MovementCurves.normalCurve(i) >= 0 && MovementCurves.normalCurve(i) <= 1);
            assert (MovementCurves.roundedSquareCurve(i) >= 0 && MovementCurves.roundedSquareCurve(i) <= 1);
            assert (MovementCurves.sinCurve(i) >= 0 && MovementCurves.sinCurve(i) <= 1);
            assert (MovementCurves.linear(i) >= 0 && MovementCurves.linear(i) <= 1);

        }

        System.out.println("maps in range checks cleared");

        assert (MovementCurves.circleCurve(.5) == 1);
        assert (MovementCurves.parametricCurve(.5) == 1);
        assert (MovementCurves.quadraticCurve(.5) == 1);
        assert (MovementCurves.normalCurve(.5) == 1);
        assert (MovementCurves.roundedSquareCurve(.5) == 1);
        assert (MovementCurves.sinCurve(.5) == 1);
        assert (MovementCurves.linear(.5) == 1);

        System.out.println("equals 1 at peak test clears");


        for (double i = 0; i <= .9; i += .1) {
            assert (MovementCurves.exponentialEaseIn(i) < MovementCurves.exponentialEaseIn(i + .1));
            assert (MovementCurves.exponentialEaseOut(i) > MovementCurves.exponentialEaseOut(i + .1));
        }
        System.out.println("solely increasing for Ease in check clear");
        System.out.println("solely decreasing for Ease out check clear");

    }
}