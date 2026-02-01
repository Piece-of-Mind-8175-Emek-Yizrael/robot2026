package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.AutoLog;

public interface climbIO {

    @AutoLog

    public static class climbIOInputs {
        public static double voltage;
        public static double output;
        public static double angle;
        public static boolean isPressed;
    }

}
