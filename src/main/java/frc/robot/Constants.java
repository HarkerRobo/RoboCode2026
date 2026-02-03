package frc.robot;

import java.util.function.Function;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.simulation.BallConstants;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

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
        
        public static final double GEAR_RATIO = 100.0; // TODO

		public static final double KP = 26.41; // TODO
		public static final double KI = 0.0; // TODO
		public static final double KD = 0.89216; // TODO
		
        public static final double KS = 0.081286; // TODO
        public static final double KV = 11.462; // TODO
        public static final double KA = 0.23849; // TODO
		public static final double KG = 0.012971; // TODO

        public static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;

        public static final double MOMENT_OF_INERTIA = 0.001; // TODO (kg m^2)

        public static final double HOOD_LENGTH = 0.5; // TODO meters
        public static final double HOOD_MIN_ANGLE = 5.0; // TODO degrees
        public static final double HOOD_MAX_ANGLE = 70.0; // TODO degrees
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

    public class Simulation
    {
        public static final double FIELD_HEIGHT = 8.069326;
        public static final double FIELD_WIDTH = 16.540988;

        // excluding steel barrier
        public static final Rectangle2d DEPOT = new Rectangle2d(
            new Translation2d(0.0, FIELD_HEIGHT - 1.570736 - 0.0762),
            new Translation2d(0.6858 - 0.0762, FIELD_HEIGHT - 1.570736 - 1.0668 + 0.0762));
        
        public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_WIDTH / 2.0, FIELD_HEIGHT / 2.0);

        public static final Function<Double, Double> ROTATE_X = (Double x) -> x + 2 * (FIELD_CENTER.getX() - x);
        public static final Function<Double, Double> ROTATE_Y = (Double y) -> y + 2 * (FIELD_CENTER.getY() - y);

        public static final Translation2d CENTER_UPPER_REFERENCE = new Translation2d(FIELD_CENTER.getX(), FIELD_CENTER.getY() + 0.0254);
        public static final Translation2d CENTER_LOWER_REFERENCE = new Translation2d(FIELD_CENTER.getX(), FIELD_CENTER.getY() - 0.0254);

        public static final double FUEL_DIAMETER = 0.15;
        public static final double MIN_FUEL_MASS = 0.203;
        public static final double MAX_FUEL_MASS = 0.227;

        public static final int TOTAL_FUEL = 504;
        public static final int FUELS_TAKEN_BY_OTHER_ROBOTS = 0;

        public static final Rectangle2d HUB_CONTENTS = new Rectangle2d(new Pose2d(new Translation2d(4.574794, 4.059936), new Rotation2d()), 1.1938, 1.1938);

        public static final Rectangle2d OUTPOST = new Rectangle2d(
            new Translation2d(-0.84 - 0.5 * FUEL_DIAMETER, 0.331 - 0.5 * FUEL_DIAMETER), 
            new Translation2d(-0.0708 + 0.5 * FUEL_DIAMETER, 1.008 + 0.5 * FUEL_DIAMETER));

        public static final Translation3d OUTPOST_SPAWN_LOCATION_LOWER = new Translation3d(0.5 * FUEL_DIAMETER, 0.2655, 0.714);
        public static final Translation3d OUTPOST_SPAWN_LOCATION_UPPER = new Translation3d(0.5 * FUEL_DIAMETER, 1.0735, 0.714);

        public static final BallConstants BALL_CONSTANTS = new BallConstants(
            (MAX_FUEL_MASS + MIN_FUEL_MASS) / 2.0,
            FUEL_DIAMETER / 2.0, 1.2, 0.30, 1.2, 0.35, 9.81, 20);

        public static final double HUB_INTAKE_HEIGHT = 1.8288;
    }
    public static final class Climb {
        public static final int ID = 17;

        public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final InvertedValue CLIMB_INVERTED = InvertedValue.Clockwise_Positive; // TODO

        public static final double ELEVATOR_GEAR_RATIO = 23.7;
        public static final double CLIMB_GEAR_RATIO = 23.7;

        public static final Per<VoltageUnit,AngleUnit> KP_ELEVATOR = Volts.of(5).per(Rotation); // TODO
        public static final double KP_CLIMB = 26.41; // TODO
        public static final double KI_ELEVATOR = 0.0; //TODO
        public static final double KI_CLIMB = 0.0; // TODO
        public static final double KD_ELEVATOR = 0.89216; //TODO
        public static final double KD_CLIMB = 0.0; // TODO

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(100 + 40); // TODO
        public static final Current ELEVATOR_STALLING_CURRENT = Amps.of(50);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(100 + 20); // TODO

        public static final Angle CLIMB_POSITION_LEVEL_1 = Rotations.of(1.0);  // rotations // TODO
        public static final Angle CLIMB_POSITION_LEVEL_2 = Rotations.of(2.0);  // rotations // TODO
        public static final Angle CLIMB_POSITION_LEVEL_3 = Rotations.of(3.0);  // rotations // TODO
		
        public static final double KS = 0.081286; // TODO
        public static final double KV = 11.462; // TODO
        public static final double KA = 0.23849; // TODO
		public static final double KG = 0.012971; // TODO
        public static final Angle MAX_ERROR = Rotations.of(0.1);

        public static final Voltage ELEVATOR_STAY_VOLTAGE = Volts.of(1.0); //TODO
        public static final Voltage ELEVATOR_GO_DOWN_VOLTAGE = Volts.of(-5.0); //TODO
    }
}
