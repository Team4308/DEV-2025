package frc.robot;

public final class Constants {
    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }
    
    public static class Mapping {
        public static class Drive {
            // Motor ID's

            // TODO: CHANGE IDS AGAIN 
            public static int Left_Front = 1;
            public static int Left_Back = 2;

            public static int Right_Front = 3;
            public static int Right_Back= 4;

            // Invert motor

            // NOTE: Front is leader everyrthing else is follower
            public static boolean Left_Front_Inverted = false;
            public static boolean Right_Front_Inverted = true;
        
            public static boolean Left_Back_Inverted = true;
            public static boolean Right_Back_Inverted = false;



        }
    }
    public static class Generic {
        public static int timeoutMs = 3000;
    }

    public static class Simulation {

        public static class Camera {
            public static int fps = 22;
            public static int AvgLatencyMs = 20;
            public static int LatencyStdDevMs = 50;
        } 

        public static final class arm {
            public static final int motorCount = 1;
            public static final double gearRatio = 500/1;        // 500 - 1 
            public static final double armLengthMeters = 0.254;   /// Roughly 22 inchs (CAD)
            public static final double armMassKg = 5.216312;          // Around 11.5 pounds (CAD)
            public static final double minAngleDeg = 0.0;
            public static final double maxAngleDeg = 180.0;
            public static final double armOffsetDeg = 0;      
            public static final boolean simulateGravity = true;
            public static final double simLoopPeriodSec = 0.020;  // 20ms loop
            public static final double atAngleToleranceDeg = 3.0;

            // Simulation Pid since its not even close to irl tuning
            public static final double kP = 0.61;
            public static final double kI = 0.32;
            public static final double kD = 0.17;
    
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

        public static final double kS = 0.0; // TODO: Use ReClac Again
        public static final double kG = 0.74;
        public static final double kV = 0.03;

        // Offset
        public static final double ZERO_OFFSET_DEG = 0.0;
        public static final double GEAR_RATIO = 500.0;
        public static final double DEG_PER_ENCODER_UNIT = 360.0 / GEAR_RATIO;

        // Profiling 
        public static final double MAX_VEL_DEG_PER_S = 120.0;
        public static final double MAX_ACC_DEG_PER_S2 = 360.0;
    }

    public static final class DriveConstants {
        // Drive Constants
        public static final double trackWidthMeters = 0.6;  // TODO: Measure
        public static final int leftEncoderChannelA = 22; 
        public static final int leftEncoderChannelB = 9;
        public static final int rightEncoderChannelA = 13;
        public static final int rightEncoderChannelB = 11;

        // Output limits
        public static final double nominalOutputForward = 0.0;
        public static final double nominalOutputReverse = 0.0;
        public static final double peakOutputForward = 1.0;
        public static final double peakOutputReverse = -1.0;

        // Max robot speeds
        public static final double maxLinearSpeedMps = 3.0;
        public static final double maxAngularSpeedRadPerSec = Math.PI;

        // Path planning constraints
        public static final double pathMaxVelMps = 3.0;
        public static final double pathMaxAccelMps2 = 3.0;
        public static final double pathMaxAngularVelRadPerSec = Math.PI;
        public static final double pathMaxAngularAccelRadPerSec2 = 3.0;

        // Controllers
        public static final double ltvUpdatePeriodSec = 0.02;

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
        public static final int minTagsForVision = 2;
        public static final double visionMaxTranslationErrorMeters = 0.35; // tune
        public static final double visionMaxYawErrorDeg = 10.0;            // tune

        // Estimator 
        public static final double stateStdDevPosMeters = 0.05;
        public static final double stateStdDevThetaDeg = 5.0;
        public static final double visionStdDevPosMeters = 0.5;
        public static final double visionStdDevThetaDeg = 20.0;

        public static final double simVisionStdDevPosMeters = 0.3;
        public static final double simVisionStdDevThetaDeg = 5.0;
        public static final double simUpdatePeriodSec = 0.05;  
        public static final double simLatencyMs = 1.0;


        public static final int coralClassId = 0; // TODO: Change to the real id 

        // Seek
        public static final double seekSteerKp = 0.8;         
        public static final double seekMaxFwd = 0.4;         
        public static final double seekMinFwd = 0.15;         
        public static final double seekAlignDeadband = 0.05;  
        public static final double seekStopArea = 0.30;       
        public static final double seekLostRotate = 0.25;     
        public static final double seekTimeoutSec = 4.0;      
    }
    public static final class Deepclimb{ 
        public static final int MOTORID = 30;
    }
}
