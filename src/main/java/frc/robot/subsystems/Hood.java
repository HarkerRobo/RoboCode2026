package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;
import frc.robot.simulation.StallSimulator;


public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX master;
    private static TalonFX follower;

    private double desiredPosition; // rotations
    
    private final SingleJointedArmSim motorSimModel = new SingleJointedArmSim(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.001, Constants.Hood.GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2)).getGearbox(),
        Constants.Hood.GEAR_RATIO,
        Constants.Hood.MOMENT_OF_INERTIA,
        Constants.Hood.LENGTH,
        Units.degreesToRadians(Constants.Hood.MIN_ANGLE),
        Units.degreesToRadians(Constants.Hood.MAX_ANGLE),
        true,
        Units.degreesToRadians(10.0)
        // no std devs -> no noise simulated
        );


    StallSimulator stallSimulator;


    /**
     * Initializes the master/follower TalonFX motors and applies configuration.
     * If simulated, sets up motor orientation and initializes the stall simulator.
    */
    private Hood()
    {
        master = new TalonFX(Constants.Hood.MASTER_ID);
        follower = new TalonFX(Constants.Hood.FOLLOWER_ID);

        config();

        if (isSimulated())
        {
            TalonFXSimState simState = master.getSimState();
            simState.Orientation = Constants.Hood.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            stallSimulator = new StallSimulator(()->master.getPosition().getValueAsDouble());
        }
    }

    /**
     * Clears sticky faults on master and follower. 
     * Makes both motors start in a fully‑configured state I guess.
     */
    private void config()
    {
        master.clearStickyFaults();
        follower.clearStickyFaults();
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (!isSimulated())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Hood.GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Hood.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Hood.MM_JERK;

        config.MotorOutput.Inverted = Constants.Hood.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        config.Slot0.kG = Constants.Hood.KG;
        config.Slot0.kV = Constants.Hood.KV;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        master.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

    }

    /**
    * Tells the hood to head toward a specific angle using Motion Magic.
    * Also remembers that angle so other code can check where it was told to go.
    */
    public void moveToPosition(Angle desiredPosition)
    {
        //System.out.println("Moving to position: " + desiredPosition.in(Degrees) + "°");
        this.desiredPosition = desiredPosition.in(Rotations);
        master.setControl(new MotionMagicVoltage(desiredPosition));
    }
    
    /**
    * Grabs the hood’s current angle straight from the motor sensor.
    * Good for anything that needs to know where the hood actually is.
    */
    public Angle getPosition()
    {
        return master.getPosition().getValue();
    }
    
    /**
    * Returns the voltage currently applied to the hood motor.
    * Useful for getting the voltage.
    */
    public Voltage getVoltage()
    {
        return master.getMotorVoltage().getValue();
    }
    
    /**
    * Returns the hood’s current angular velocity.
    * Used if you need the velocity.
    */
    public AngularVelocity getVelocity()
    {
        return master.getVelocity().getValue();
    }

    /**
    * Directly sets the internal motor position value.
    * Only runs when the subsystem isn’t disabled.
    */
    public void setPosition(Angle position)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        master.setPosition(position);
    }

    /**
    * Commands the hood to hold a target velocity.
    * Automatically does feedforward and PID.
    */
    public void setVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        master.setControl(new VelocityVoltage(velocity));
    }

    /**
    * Drives the motor with a raw percent output.
    * Blocked when disabled.
    */
    public void setDutyCycle(double dutyCycle)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        master.setControl(new DutyCycleOut(dutyCycle));
    }

    /**
    * Pushes a specific voltage into the motor.
    * Blocked if disabled
    */
    public void setVoltage(Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        master.setVoltage(voltage.in(Volts));
    }

    
    /**
    * Checks if the hood is at the angle it was told to reach.
    * Returns true when it’s there.
    */
    public boolean readyToShoot ()
    {
        return Math.abs(master.getPosition().getValue().in(Rotations) - desiredPosition) < Constants.EPSILON;
    }
    
    /**
    * Returns the last commanded hood angle.
    * See where what its aiming for
    */
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }
    
    /**
    * Detects whether the hood is drawing stall‑level current.
    * In simulation, also checks the StallSimulator for virtual stalls.
    */
    public boolean isStalling()
    {
        if (isSimulated() && stallSimulator.get()) return true;
        return Math.abs(master.getStatorCurrent().getValueAsDouble()) >= Constants.Hood.STALLING_CURRENT;
    }


    /**
    * Updates the simulated hood physics when running in simulation mode.
    * Feeds simulated rotor position/velocity back into the TalonFXSimState.
    */
    @Override
    public void periodic()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = master.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage motorVoltage = simState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            motorSimModel.setInputVoltage(motorVoltage.in(Volts));
            motorSimModel.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(
                    Units.radiansToRotations(motorSimModel.getAngleRads()) * Constants.Hood.GEAR_RATIO);
            simState.setRotorVelocity(
                    Units.radiansToRotations(motorSimModel.getVelocityRadPerSec()) * Constants.Hood.GEAR_RATIO);
        }
    }

    /**
    * Returns true if the subsystem is marked
    * as simulated in RobotContainer.
    */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOOD_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
    * Checks if the subsystem is
    * marked as disabled. 
    */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOOD_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.05).div(Seconds.of(1.)),Volts.of(0.1),Seconds.of(10.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->master.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Hood")
                .voltage(getVoltage())
                .angularPosition(getPosition())
                .angularVelocity(getVelocity()),
        this)
    );

    /** 
     * Creates ramp test for SysId characterization.
     * Gather data for feedforward tuning.
     */
    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
    * Creates an acceleration test 
    * for SysId.
    */
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    

    /**
    * Returns the Hood subsystem instance.
    * Makes sure there isnt a second created
    */
    public static Hood getInstance()
    {
        if (instance == null) instance = new Hood();
        return instance;
    }
}
