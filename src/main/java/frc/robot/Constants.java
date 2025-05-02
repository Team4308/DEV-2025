package frc.robot;

public final class Constants {
    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }


    public static final class Intake {
        public static final int Intake_Motor_id = 1;
    }

    public static final class Vision{
        // camera offest in meters and degrees
        public static final double cam_offset_front = 0;
        public static final double cam_offset_side = 0;
        public static final double cam_offset_up = 0;
        public static final double cam_offset_pitch = 0;
        public static final double cam_offset_roll = 0;
        public static final double cam_offset_yaw = 0;
        
        public static final double ideal_detection_range = 10;
    }
}
