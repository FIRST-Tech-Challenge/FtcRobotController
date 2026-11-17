package org.firstinspires.ftc.teamcode.Swerve.Geo;
    public class D2Vector {
        public double x,y;

        public D2Vector(double x, double y){
            this.x = x;
            this.y = y;
        }

        public static D2Vector fromHeadAndMagnitude(double h, double m){
            return new D2Vector(Math.cos(h) * m, Math.sin(h) * m);
        }

        public D2Vector scalarmult(double other) {
            return new D2Vector(x * other, y * other);
        }

        public D2Vector scalardiv(double scalar) {
            return new D2Vector(x / scalar, y / scalar);
        }

        public D2Vector subt(D2Vector other) {
            return new D2Vector(x - other.x, y - other.y);
        }

        public double dotprod(D2Vector other) {
            return x * other.x + y * other.y;
        }

        public double magnitude() {
            return Math.hypot(x, y);
        }

        public D2Vector unit() {
            return this.scalardiv(magnitude());
        }

        public D2Vector rotate(double angle) {
            return new D2Vector(
                    x * Math.cos(angle) - y * Math.sin(angle),
                    x * Math.sin(angle) + y * Math.cos(angle));
        }

        public double crossprod(D2Vector other) {
            return x * other.y - y * other.x;
        }

    }
