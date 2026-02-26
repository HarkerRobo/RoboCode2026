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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VoltageOut;

public class Indexer extends SubsystemBase
{
    
    private static Indexer instance;

    private static TalonFX motor;
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Indexer.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

    /**
     * Creates the indexer motor and applies all configuration settings.
     * If running in simulation, sets up the TalonFX sim state so the virtual motor behaves correctly.
     */
    private Indexer() { 
        motor = new TalonFX(Constants.Indexer.ID);
        config();
        
        if (isSimulated())
        {
            motor.getSimState().Orientation = Constants.Indexer.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX settings: PID values, current limits, inversion, voltage limits, and sensor ratio.
     * Ensures the indexer motor starts with the correct behavior before anything commands it.
     */
    private void config() {
        motor.clearStickyFaults();

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Indexer.KS;
        slot0Configs.kV = Constants.Indexer.KV;
        slot0Configs.kA = Constants.Indexer.KA;
        slot0Configs.kP = Constants.Indexer.KP;
        slot0Configs.kI = Constants.Indexer.KI;
        slot0Configs.kD = Constants.Indexer.KD;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.Indexer.STATOR_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.Inverted = Constants.Indexer.INVERTED;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        talonFXConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        talonFXConfigs.Feedback.SensorToMechanismRatio = Constants.Indexer.GEAR_RATIO;


        /* // add if necessary
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Indexer.MM_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.MM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Indexer.MM_JERK;
        */

        motor.getConfigurator().apply(talonFXConfigs);
    }

    /**
     * Commands the motor to spin at the given velocity.
     * If the subsystem is disabled, the command is ignored.
     */
    public void setVelocity(AngularVelocity velocity) {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        motor.set(velocity.in(RotationsPerSecond));
    }

    /**
     * Returns the motor’s current angular velocity.
     * Check how fast the indexer is actually moving.
     */
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    /**
     * Sends a specific voltage to the motor.
     * Ignored when the subsystem is disabled.
     */
    public void setVoltage(Voltage voltage) {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        motor.setVoltage(voltage.in(Volts));
    }

    /**
     * Returns the voltage currently applied to the motor.
     * Check what the controller is outputting.
     */
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    /**
     * Returns the raw percent output
     * the motor is running at.
     */
    public double getDutyCycle()
    {
        return motor.getDutyCycle().getValueAsDouble();
    }
    
    /**
     * When in simulation, updates the DCMotorSim and 
     * writes the new rotor state into the TalonFX sim.
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
            simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Indexer.GEAR_RATIO));
            simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Indexer.GEAR_RATIO));
        }
    }

    /**
     * Checks RobotContainer to see if this subsystem is running in simulation mode.
     * Controls whether the sim model runs.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Checks if the subsystem is marked disabled.
     * Blocks all motor commands when it is.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Indexer")
                .voltage(getVoltage())
                .angularPosition(motor.getPosition().getValue())
                .angularVelocity(getVelocity()),
        this)
    );

    /**
     * Creates a SysId ramp test.
     * Logs voltage, position, and velocity for tuning.
     */
    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
     * Creates a fast SysId acceleration test.
     * Data for motor model tuning.
     */
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }

    /**
     * Returns the single Indexer instance.
     * Prevents multiple copies from being created.
     */
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }
}
