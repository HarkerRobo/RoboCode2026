package frc.robot.simulation;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Util;

import static frc.robot.Constants.Simulation.*;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Stores the position of fuels and models their physics/possession, making sure that their associated poses line up to their position for correct visualization
 */
public class SimulationState
{
    /**
     * What possesses the fuel and modifies its position; Field means that the fuel has its own position modified by physics; anything else means the position is subject
     * to change to display it visually in the logical location
     */
    public enum FieldLocation
    {
        Field,
        BlueOutpost,
        RedOutpost,
        BlueHub,
        RedHub,
        Robot,
        OtherRobot
    }
    public static class FuelPosition
    {
        public FieldLocation location;

        public BallState state; // associated physics values such as position, velocity, spin

        public FuelPosition(FieldLocation location, double xPositionMeters, double yPositionMeters)
        {
            this.location = location;
            state = new BallState(new Pose3d(new Translation3d(xPositionMeters, yPositionMeters, 0.5 * FUEL_DIAMETER), new Rotation3d()), Translation3d.kZero, Translation3d.kZero);
        }
        
        public FuelPosition(FieldLocation location, double xPositionMeters, double yPositionMeters, double zPositionMeters)
        {
            this.location = location;
            state = new BallState(new Pose3d(new Translation3d(xPositionMeters, yPositionMeters, zPositionMeters), new Rotation3d()), Translation3d.kZero, Translation3d.kZero);
        }

        /*
        public static class FuelPositionStruct implements Struct<FuelPosition>
        {
            public FuelPositionStruct () {}

            @Override
            public Class<FuelPosition> getTypeClass() 
            {
                return FuelPosition.class;
            }

            @Override
            public String getTypeName() 
            {
                return "FuelPosition";
            }

            @Override
            public int getSize() 
            {
                return Translation3d.struct.getSize() + 4;
            }

            @Override
            public String getSchema() 
            {
                return "Translation3d t;FieldLocation l";
            }

            @Override
            public FuelPosition unpack(ByteBuffer bb) 
            {
                Translation3d t = Translation3d.struct.unpack(bb);
                FieldLocation l =  FieldLocation.values()[bb.getInt()];
                return new FuelPosition(l, t.getX(), t.getY(), t.getZ());
            }

            @Override
            public void pack(ByteBuffer bb, FuelPosition value)
            {
                Translation3d.struct.pack(bb, value.position);
                bb.putInt(value.location.ordinal());
            }

            @Override
            public boolean isImmutable() 
            {
                return false;
            }
        }
        */
    }

    public long lastTime = 0;

    public FuelPosition[] fuelPositions = new FuelPosition[TOTAL_FUEL];
    public Translation3d[] fuelPositionsRaw = new Translation3d[fuelPositions.length];

    public int fuelsInRobot = 0;
    public int fuelsInBlueHub = 0;
    public int fuelsInRedHub = 0;
    public int fuelsInBlueOutpost = 0;
    public int fuelsInRedOutpost = 0;

    private SimulationState ()
    {
        init();
    }

    /**
     * Sets the position of fuels to their position on the field at the start of the match
     */
    public void init ()
    {
        int offset = 0;

        {
            double dx = (DEPOT.getXWidth() - FUEL_DIAMETER) / 3;
            double dy = (DEPOT.getYWidth() - FUEL_DIAMETER) / 5;

            double x0 = DEPOT.getCenter().getX() - 0.5 * DEPOT.getXWidth() + 0.5 * FUEL_DIAMETER;
            double y0 = DEPOT.getCenter().getY() - 0.5 * DEPOT.getYWidth() + 0.5 * FUEL_DIAMETER;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    fuelPositions[offset + i * 6 + j] = new FuelPosition(FieldLocation.Field, x0 + i * dx, y0 + j * dy);
                    fuelPositions[offset + i * 6 + j + 24] = new FuelPosition(FieldLocation.Field, ROTATE_X.apply(x0 + i * dx), ROTATE_Y.apply(y0 + j * dy));
                }
            }
            offset += 48;
        }

        for (int i = 0; i < 24; i++)
        {
            fuelPositions[offset + i] = new FuelPosition(FieldLocation.BlueOutpost, -1.0, -1.0);
            fuelPositions[offset + i + 24] = new FuelPosition(FieldLocation.RedOutpost, -1.0, -1.0);
        }
        offset += 48;

        for (int i = 0; i < 8; i++)
        {
            fuelPositions[offset + i] = new FuelPosition(FieldLocation.Robot, -1.0, -1.0);
        }
        offset += 8;

        for (int i = 0; i < FUELS_TAKEN_BY_OTHER_ROBOTS; i++) 
        {
            fuelPositions[offset + i] = new FuelPosition(FieldLocation.OtherRobot, -1.0, -1.0);
        }
        offset += FUELS_TAKEN_BY_OTHER_ROBOTS;


        {
            double x0 = CENTER_UPPER_REFERENCE.getX() + 0.5 * FUEL_DIAMETER;
            double y0u = CENTER_UPPER_REFERENCE.getY() + 0.5 * FUEL_DIAMETER;
            double y0l = CENTER_LOWER_REFERENCE.getY() - 0.5 * FUEL_DIAMETER;

            double dx = FUEL_DIAMETER;
            double dy = FUEL_DIAMETER;

            for (int i = 0; offset < fuelPositions.length; i++) 
            {
                // upper
                for (int j = 0; j < 12 && offset < fuelPositions.length; j++) 
                {
                    fuelPositions[offset] = new FuelPosition(FieldLocation.Field, x0 + dx * ((j % 2 == 0) ? (j / 2) : -1 - (j / 2)), y0u + dy * i);
                    offset++;
                }

                // lower
                for (int j = 0; j < 12 && offset < fuelPositions.length; j++) 
                {
                    fuelPositions[offset] = new FuelPosition(FieldLocation.Field, x0 + dx * ((j % 2 == 0) ? (j / 2) : -1 - (j / 2)), y0l - dy * i);
                    offset++;
                }
            }
        }


    }

    public void update ()
    {
        StallSimulator.update();

        long now = RobotController.getFPGATime();
        double dt = (now - lastTime) * 1e-6;

        Pose2d robotPose = Robot.instance.robotContainer.drivetrain.samplePoseAt(now).get();
        double RobotX = robotPose.getTranslation().getX();
        double RobotY = robotPose.getTranslation().getY();
        double RobotTheta = robotPose.getRotation().getRadians();

        fuelsInRobot = 0;
        fuelsInBlueHub = 0;
        fuelsInRedHub = 0;
        fuelsInBlueOutpost = 0;
        fuelsInRedOutpost = 0;
        for (int i = 0; i < fuelPositions.length; i++)
        {

            switch (fuelPositions[i].location)
            {
                case Robot: fuelsInRobot++; break;
                case BlueHub:
                    fuelPositions[i].state.pose = new Pose3d(Util.packEl(HUB_CONTENTS, 0.0762, FUEL_DIAMETER, fuelsInBlueHub), new Rotation3d());
                    fuelsInBlueHub++;
                    break;
                case RedHub:
                    fuelPositions[i].state.pose = new Pose3d(Util.packEl(Util.rotate(HUB_CONTENTS), 0.0762, FUEL_DIAMETER, fuelsInRedHub), new Rotation3d());
                    fuelsInRedHub++;
                    break;
                case BlueOutpost:
                    fuelPositions[i].state.pose = new Pose3d(Util.packEl(OUTPOST, 0.0762, FUEL_DIAMETER, fuelsInBlueOutpost), new Rotation3d());
                    fuelsInBlueOutpost++;
                    break;
                case RedOutpost: 
                    fuelPositions[i].state.pose = new Pose3d(Util.packEl(Util.rotate(OUTPOST), 0.0762, FUEL_DIAMETER, fuelsInRedOutpost), new Rotation3d());
                    fuelsInRedOutpost++; 
                    break;
                case Field: // if the fuel is in the field, change the position depending on physics and setting the fuel location if the fuel is in the correct position
                    if (lastTime != 0)
                    {
                        BallPhysics.step(fuelPositions[i].state, BALL_CONSTANTS, dt);
                    }

                    if (fuelPositions[i].state.pose.getY() - RobotY - (fuelPositions[i].state.pose.getX() - RobotX - (Constants.ROBOT_DIAMETER / (2 * Math.sin(RobotTheta))))/Math.tan(RobotTheta) <= 0.1) //Elijah's Formula (will double check with him)
                    {
                        fuelPositions[i].location = FieldLocation.Robot;
                    }

                    if (Math.abs(fuelPositions[i].state.pose.getZ() - HUB_INTAKE_HEIGHT) <= FUEL_DIAMETER && Util.within(HUB_CONTENTS, fuelPositions[i].state.pose.getTranslation()))
                    {
                        fuelPositions[i].location = FieldLocation.BlueHub;
                    }
                    
                    else if (Math.abs(fuelPositions[i].state.pose.getZ() - HUB_INTAKE_HEIGHT) <= FUEL_DIAMETER && Util.within(Util.rotate(HUB_CONTENTS), fuelPositions[i].state.pose.getTranslation()))
                    {
                        fuelPositions[i].location = FieldLocation.BlueHub;
                    }

                    break;
                default: break;
            }

            fuelPositionsRaw[i] = fuelPositions[i].state.pose.getTranslation();
        }

        lastTime = now;
    }

    public int fuelsInRobot ()
    {
        int count = 0;
        for (FuelPosition fuelPosition : fuelPositions)
        {
            if (fuelPosition.location == FieldLocation.Robot) count++;
        }
        return count;
    }

    /**
     * Moves a fuel which is currently on the field to the specified location. For example, if the passed in location is the red hub, the fuel will be moved to to the red
     * hub, and it will be moved to the correct position for its ball state at the next call to update
     */
    public void testPlop (FieldLocation fl)
    {
        for (int i = 0; i < fuelPositions.length; i++)
        {
            if (fuelPositions[i].location == FieldLocation.Field)
            {
                fuelPositions[i].location = fl;
                break;
            }
        }
    }

    /**
     * Moves all fuel which are currently on the field to a random spot on the field, elevated by 4 meters
     */
    public void testDrop ()
    {
        for (int i = 0; i < fuelPositions.length; i++)
        {
            if (fuelPositions[i].location == FieldLocation.Field)
            {
                fuelPositions[i].state.pose = new Pose3d(new Translation3d(Math.random() * FIELD_WIDTH, Math.random() * FIELD_HEIGHT, 4.0), new Rotation3d());
                fuelPositions[i].state.velocity = Translation3d.kZero;
                fuelPositions[i].state.omega = Translation3d.kZero;
            }
        }
    }

    /**
     * Moves a fuel in the specified outpost to the field immediately outside of it
     */
    public void spawnFromOutpost(Alliance alliance)
    {
        for (FuelPosition p : fuelPositions)
        {
            if (p.location == (alliance == Alliance.Blue ? FieldLocation.BlueOutpost : FieldLocation.RedOutpost))
            {
                double random = Math.random();

                p.location = FieldLocation.Field;
                p.state.pose = new Pose3d(new Translation3d(
                    OUTPOST_SPAWN_LOCATION_LOWER.getX(), 
                    random * OUTPOST_SPAWN_LOCATION_LOWER.getY() + (1 - random) * OUTPOST_SPAWN_LOCATION_UPPER.getY(),
                    OUTPOST_SPAWN_LOCATION_LOWER.getZ()), new Rotation3d());

                if (alliance == Alliance.Red)
                {
                    p.state.pose = new Pose3d(new Translation3d(ROTATE_X.apply(p.state.pose.getX()), ROTATE_Y.apply(p.state.pose.getY()), p.state.pose.getZ()), new Rotation3d());
                }

                break;
            }
        }
    }


    private static SimulationState instance;

    public static SimulationState getInstance()
    {
         if (instance == null) instance = new SimulationState();
         return instance;
    }
}
