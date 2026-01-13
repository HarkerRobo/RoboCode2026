package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class Constants 
{
    public static final double MAX_VOLTAGE = 12.0;
    public class Turret
    {
        public static final double ERROR_THRESHOLD = 1.0; // TODO degrees

        public static final int YAW_MOTOR_ID = 0; // TODO
        public static final int PITCH_MOTOR_ID = 2; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_YAW_CRUISE_VELOCITY = 60.0;
		public static final double MM_YAW_ACCELERATION = 60.0;
		public static final double MM_YAW_JERK = 240.0;
		public static final InvertedValue YAW_INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double YAW_GEAR_RATIO = 1.0; // TODO

		public static final double YAW_KP = 1.0; // TODO
		public static final double YAW_KI = 0.1; // TODO
		public static final double YAW_KD = 0.0; // TODO
		
        public static final double YAW_KV = 0.0; // TODO
		public static final double YAW_KG = 0.0; // TODO

        public static final double YAW_FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double YAW_REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO


		public static final double MM_PITCH_CRUISE_VELOCITY = 60.0;
		public static final double MM_PITCH_ACCELERATION = 60.0;
		public static final double MM_PITCH_JERK = 240.0;
		public static final InvertedValue PITCH_INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double PITCH_GEAR_RATIO = 1.0; // TODO

		public static final double PITCH_KP = 1.0; // TODO
		public static final double PITCH_KI = 0.1; // TODO
		public static final double PITCH_KD = 0.0; // TODO
		
        public static final double PITCH_KV = 0.0; // TODO
		public static final double PITCH_KG = 0.0; // TODO

		public static final double PITCH_FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double PITCH_REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO

        public static final double MAX_PITCH = 90.0; // TODO degrees

    }
}
