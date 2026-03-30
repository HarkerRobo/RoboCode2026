
package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

/**
 * This class controls the powering of the flywheels to propel the fuel
 */
public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    private TalonFX master;
    private TalonFX follower;
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(2));
    
    private double targetVelocity = 0.0;

    /**
     * Creates the Shooter subsystem and configures the master and follower motors.
     * Initializes simulation settings.
     */
    private Shooter()
    {
        master = new TalonFX(Constants.Shooter.MASTER_ID, Constants.CAN_SUPERSTRUCTURE);
        follower = new TalonFX(Constants.Shooter.FOLLOWER_ID, Constants.CAN_SUPERSTRUCTURE);

        config();
        
        if (isSimulated())
        {
            master.getSimState().Orientation = Constants.Shooter.MASTER_MECHANICAL_ORIENTATION;
            master.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            follower.getSimState().Orientation = Constants.Shooter.FOLLOWER_MECHANICAL_ORIENTATION;
            follower.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        
    }

    /**
     * Applies all TalonFX settings: PID, feedforward, inversion, limits, and sensor ratio.
     */
    private void config()
    {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        if (isSimulated())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Shooter.GEAR_RATIO;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;
        
        config.Slot0.kP = Constants.Shooter.KP;
        config.Slot0.kI = Constants.Shooter.KI;
        config.Slot0.kD = Constants.Shooter.KD;
        config.Slot0.kS = Constants.Shooter.KS;
        config.Slot0.kV = Constants.Shooter.KV;
        config.Slot0.kA = Constants.Shooter.KA;
        config.MotorOutput.Inverted = Constants.Shooter.INVERTED;

        master.getConfigurator().apply(config);

        follower.setControl(new Follower(Constants.Shooter.MASTER_ID, Constants.Shooter.MOTOR_ALIGNMENT));
    }
    
    /**
     * @return motor voltage
     */
    public Voltage getVoltage()
    {
        return master.getMotorVoltage().getValue();
    }
    /**
     * @return velocity of shooter
     */
    public AngularVelocity getVelocity()
    {
        return master.getVelocity().getValue();
    }
    
    /**
     * Returns the linear surface velocity of the flywheel.
     * Converts angular velocity into tangential speed at the wheel edge.
     */
    public LinearVelocity getEffectiveVelocity()
    {
        return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }

    /**
     * Returns the target linear surface velocity.
     */
    public LinearVelocity getTargetEffectiveVelocity()
    {
        return MetersPerSecond.of(targetVelocity * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }
    
    /**
     * Returns the target angular velocity of the shooter.
     */
    public AngularVelocity getTargetVelocity()
    {
        return RotationsPerSecond.of(targetVelocity);
    }

    /**
     * Sets both flywheels to the same angular velocity.
     * Calls the individual left and right setters internally.
     */
    public void setVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        master.setControl(new VelocityVoltage(velocity));
        targetVelocity = velocity.in(RotationsPerSecond);
    }
        
    /**
     * Sets the flywheel velocity using linear surface speed.
     * Converts the request into angular velocity internally.
     */
    public void setEffectiveVelocity(LinearVelocity velocity)
    {
        setVelocity(RotationsPerSecond.of(velocity.in(MetersPerSecond) / Constants.Shooter.FLYWHEEL_CIRCUMFERANCE));
    }

    /**
     * Applies a direct voltage to both flywheel motors.
     * Blocks the command when the subsystem is disabled.
     */
    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        master.setControl(new VoltageOut(voltage));
    }

    /**
     * Drives both flywheels using a raw duty cycle percentage.
     * Blocks the command when the subsystem is disabled.
     */
    public void setDutyCycle (double dutyCycle)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        master.setControl(new DutyCycleOut(dutyCycle));
    }
    
    
    /**
     * Returns true when both flywheels are within tolerance of their targets.
     * Used to determine when the shooter is stable enough to fire.
     */
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState masterSimState = master.getSimState();

            // set the supply voltage of the TalonFX
            masterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            sim.setInputVoltage(masterSimState.getMotorVoltageMeasure().in(Volts));
            sim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            masterSimState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
            masterSimState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));
        }
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->master.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
                .voltage(getVoltage())
                .angularPosition(master.getPosition().getValue())
                .angularVelocity(getVelocity()),
        this)
    );
    
    /**
     * Creates a quasistatic SysId test for the left flywheel.
     * Used to characterize feedforward constants.
     */
    public boolean readyToShoot()
    {
        return Math.abs(master.getVelocity().getValue().in(Rotations.per(Second)) - targetVelocity) < Constants.Shooter.MAX_ERROR;
    }
    
    /**
     * Creates a quasistatic SysId test command for the shooter.
     * Characterize feedforward constants at low acceleration.
     */
    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
     * Creates a dynamic SysId test command for the shooter.
     * Measure acceleration response for tuning.
     */
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId D" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }


    /**
     * Returns true if the subsystem is running in simulation mode.
     * Controls whether the DCMotorSim models are updated.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Returns true if the subsystem is disabled.
     * Prevents all motor commands from being applied.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEX) == SubsystemStatus.Disabled;
    }
    
    /**
     * Returns the Shooter subsystem instance.
     * Ensures only one instance is ever created.
     */
    public static Shooter getInstance()
    {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}                 
