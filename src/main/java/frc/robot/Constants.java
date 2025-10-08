package frc.robot;

public final class Constants {
    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }
    
    public static class Mapping {
        public static class Drive {
            // Motor ID's
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

    public static class Simulation {

        public static class Camera {
            // Close to LL2 ( Might be slower )
            public static int fps = 22;
            public static int AvgLatencyMs = 20;
            public static int LatencyStdDevMs = 50;
        } 

    }


    public static final class Intake {
        public static final int Intake_Motor_id = 6;
        public static final int Intake_Sensor_id = 7;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 20;

        public static final double SCORING_ANGLE = 35.0;
        public static final double RESTING_ANGLE = 15.0;
        public static final double FEEDER_ANGLE = 90.0;
        public static final double  GROUND_ANGLE = 0.0;
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;
        public static final double kG = 0.5;
        public static final double kV = 0.0;
    }

    public static final class DriveConstants {
        // Drive Constants
        public static final double trackWidthMeters = 0.6; 
        public static final int leftEncoderChannelA = 22; 
        public static final int leftEncoderChannelB = 9;
        public static final int rightEncoderChannelA = 13;
        public static final int rightEncoderChannelB = 11;

        // Tolerances
        public static final int HeadingTolerance = 3; // degrees
    }

    public static final class Vision {
        public static final String LIMELIGHT_TABLE_NAME = "limelight";
        public static final double camOffsetFront = 0;
        public static final double camOffsetSide = 0;
        public static final double camOffsetUp = 0;
        public static final double camOffsetPitch = 0;
        public static final double camOffsetRoll = 0;
        public static final double camOffsetYaw = 0;
        public static final double idealDetectionRange = 10;
        public static final double maxGyroRate = 360.0;
    }
}
