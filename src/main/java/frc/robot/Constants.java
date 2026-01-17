package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class Constants 
{
    public static final double MAX_VOLTAGE = 12.0;
    
    public static final double EPSILON = 0.001;

    public class Shooter
    {
        public static final int MOTOR_ID = 3;
        
        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 60.0;
		public static final double MM_ACCELERATION = 60.0;
		public static final double MM_JERK = 240.0;
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 1.0; // TODO
		public static final double KI = 0.1; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KV = 0.0; // TODO
		public static final double KG = 0.0; // TODO

        public static final double DEFAULT_VELOCITY = 0.1; // TODO rotations per second (nonzero to decrease startup time)
        public static final double SHOOT_VELOCITY = 10.0; // TODO rotations per second
    }
    
    public class Hood
    {
        public static final int MOTOR_ID = 4;
        
        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 1.0; // TODO
		public static final double MM_ACCELERATION = 5.0;  // TODO
		public static final double MM_JERK = 10.0; // TODO
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 1.0; // TODO
		public static final double KI = 0.1; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KV = 0.0; // TODO
		public static final double KG = 0.0; // TODO

        public static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO
    }

    public class Turret
    {
        public static final double ERROR_THRESHOLD = 1.0; // TODO degrees

        public static final int MOTOR_ID = 0; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 60.0;
		public static final double MM_ACCELERATION = 60.0;
		public static final double MM_JERK = 240.0;
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 1.0; // TODO
		public static final double KI = 0.1; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KV = 0.0; // TODO
		public static final double KG = 0.0; // TODO

        public static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO
    }
}
