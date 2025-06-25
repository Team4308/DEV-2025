package frc.robot;

public final class Constants {
    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }
    
    public static class Mapping {
        public static class Drive {
            public static int Left = 0;
            public static int Right = 1;
            public static int Left1 = 2;
            public static int Left2 = 3;
            public static int Right1 = 4;
            public static int Right2= 5;
        }
    }
    public static class Generic {
        public static int timeoutMs = 3000;
    }


    public static final class Intake {
        public static final int Intake_Motor_id = 1;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 10;

        public static final double SCORING_ANGLE = 30.0;
        public static final double RESTING_ANGLE = 0.0;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;
        public static final double kG = 0.5;
        public static final double kV = 0.0;
    }
}
