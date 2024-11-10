package com.kalipsorobotics.modules;

    public enum OuttakePositions {

        ZERO_POSITION(0),
        WALL_HEIGHT(12),
        LOW_BAR(20),
        HIGH_BAR(36),
        HIGH_BASKET(43);

        private final double height;

        OuttakePositions(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

        public double getTargetTicks(double ticksPerInch) {
            return height * ticksPerInch;
        }
    }
