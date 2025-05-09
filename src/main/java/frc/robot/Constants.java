package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

public final class Constants {
    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }


    public static final class Intake {
        public static final int Intake_Motor_id = 1;
    }

    public static final class Vision{
        public static final String LIMELIGHT_TABLE_NAME = "limelight";
        public static final double idealDetectionRange = 10;
        public static final double maxGyroRate = 360;

        // camera offest in meters and degrees
        public static final double camOffsetFront = 0;
        public static final double camOffsetSide = 0;
        public static final double camOffsetUp = 0;
        public static final double camOffsetRoll = 0;
        public static final double camOffsetPitch = 0;
        public static final double camOffsetYaw = 0;
    }

    public static final class DriveConstants{
        public static final double trackWidthMeters = 0;    //filler
        public static final int leftEncoderChannelA = 0; // filler;
        public static final int leftEncoderChannelB = 0; // filler;
        public static final int rightEncoderChannelA = 0; // filler;
        public static final int rightEncoderChannelB = 0; // filler;
    }
}
