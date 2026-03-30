package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class ShooterIndexer extends SubsystemBase
{
    private static ShooterIndexer instance;

    private static TalonFX motor;
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.ShooterIndexer.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

    /**
     * Creates the ShooterIndexer subsystem and configures the TalonFX motor.
     * Initializes simulation settings.
     */
    private ShooterIndexer()
    {
        motor = new TalonFX(Constants.ShooterIndexer.ID, Constants.CAN_SUPERSTRUCTURE);
        config();
        
        if (isSimulated())
        {
            motor.getSimState().Orientation = Constants.ShooterIndexer.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX settings: PID values, current limits, inversion, voltage limits, and sensor ratio.
     * Ensures the indexer motor starts with the correct behavior before anything commands it.
     */
    private void config() 
    {
        motor.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        // not used currently
        config.Slot0.kP = Constants.ShooterIndexer.KP;
        config.Slot0.kI = Constants.ShooterIndexer.KI;
        config.Slot0.kD = Constants.ShooterIndexer.KD;

        config.CurrentLimits.StatorCurrentLimit = Constants.ShooterIndexer.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        config.CurrentLimits.SupplyCurrentLimit = Constants.ShooterIndexer.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = Constants.ShooterIndexer.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        config.Feedback.SensorToMechanismRatio = Constants.ShooterIndexer.GEAR_RATIO;

        motor.getConfigurator().apply(config);
    }

    /**
     * Returns the current angular velocity of the shooter indexer motor.
     */
    public AngularVelocity getVelocity() 
    {
        return motor.getVelocity().getValue();
    }

    /**
     * Returns the voltage applied to the shooter indexer motor.
     */
    public Voltage getVoltage() 
    {
        return motor.getMotorVoltage().getValue();
    }
    

    /**
     * Commands the motor to spin at the given velocity.
     * If the subsystem is disabled, the command is ignored.
     */
    public void setVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to ShooterIndexer");
            return;
        }
        motor.setControl(new VelocityVoltage(velocity.in(RotationsPerSecond)));
    }

    /**
     * Sends a specific voltage to the motor.
     * Direct voltage control for testing or SysId.
     */
    public void setVoltage(Voltage voltage) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to ShooterIndexer");
            return;
        }
        motor.setVoltage(voltage.in(Volts));
    }
    
    /**
     * Updates the simulated indexer physics when running in simulation mode.
     * Feeds simulated rotor position and velocity back into the TalonFXSimState.
     */
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            sim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
            sim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.ShooterIndexer.GEAR_RATIO));
            simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.ShooterIndexer.GEAR_RATIO));
        }
    }

    /**
     * Checks RobotContainer to see if this 
     * subsystem is running in simulation model
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEXER_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Returns true if the subsystem is disabled.
     * Blocks all motor commands when it is.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEXER_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("ShooterIndexer")
                .voltage(getVoltage())
                .angularPosition(motor.getPosition().getValue())
                .angularVelocity(getVelocity()),
        this)
    );

    /**
     * Creates a quasistatic SysId test for the shooter indexer.
     * Used to characterize feedforward constants.
     */
    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
     * Creates a dynamic SysId test for the shooter indexer.
     * Used to measure acceleration response for tuning.
     */
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId D" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }

    /**
     * Singleton code
     */
    public static ShooterIndexer getInstance() {
        if (instance == null) instance = new ShooterIndexer();
        return instance;
    }
}
