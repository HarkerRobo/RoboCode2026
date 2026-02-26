package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

public class IntakeExtension extends SubsystemBase 
{
    private static IntakeExtension instance;
    private TalonFX motor;

    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Climb.ELEVATOR_GEAR_RATIO),
            DCMotor.getKrakenX60(1), Constants.IntakeExtension.MIN_HEIGHT, Constants.IntakeExtension.MAX_HEIGHT, false, Constants.IntakeExtension.MIN_HEIGHT);

    /**
     * Creates the extension motor and applies all configuration settings.
     * If running in simulation, sets up the TalonFX sim state so the virtual mechanism behaves correctly.
     */
    private IntakeExtension()
    {
        motor = new TalonFX(Constants.IntakeExtension.ID);
        config();
        
        if (isSimulated())
        {
            motor.getSimState().Orientation = Constants.IntakeExtension.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX settings: inversion, PID values, current limits, voltage limits, and sensor ratio.
     * Makes sure that the extension motor starts with the correct behavior before anything commands it.
     */
    private void config()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        extensionConfig.Feedback.SensorToMechanismRatio = Constants.IntakeExtension.GEAR_RATIO;

        extensionConfig.MotorOutput.Inverted = Constants.IntakeExtension.INVERTED;
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        extensionConfig.Slot0.kP = Constants.IntakeExtension.KP;
        extensionConfig.Slot0.kI = Constants.IntakeExtension.KI;
        extensionConfig.Slot0.kD = Constants.IntakeExtension.KD;
        extensionConfig.Slot0.kS = Constants.IntakeExtension.KS;
        extensionConfig.Slot0.kV = Constants.IntakeExtension.KV;
        extensionConfig.Slot0.kA = Constants.IntakeExtension.KA;

        extensionConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        extensionConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.STATOR_CURRENT_LIMIT;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.SUPPLY_CURRENT_LIMIT;

        motor.getConfigurator().apply(extensionConfig);
    }

    /**
     * Returns the voltage
     * currently being applied to the motor.
     */
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }

    /**
     * Returns the motor’s current angular velocity.
     * Lets you see how fast the extension is moving.
     */
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }

    /**
     * Returns the motor’s current position reading.
     * Gives you the extension height.
     */
    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }

    /**
     * Sends a specific voltage to the motor.
     * Ignored if the subsystem is disabled.
     */
    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to IntakeExtension");
            return;
        }
        motor.setControl(new VoltageOut(voltage));
    }

    /**
     * Commands the motor to hold a specific speed.
     * Blocked when the subsystem is disabled.
     */
    public void setVelocity (AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to IntakeExtension");
            return;
        }
        motor.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Drives the motor with a raw percent output.
     * If the subsystem is disabled, the command is ignored.
     */
    public void setDutyCycle(double velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to IntakeExtension");
            return;
        }
        motor.setControl(new DutyCycleOut(velocity));
    }

    /**
     * Checks the stator current.
     * Returns true when the current crosses the configured threshold.
     */
    public boolean isStalling()
    {
        System.out.println("Stator Current: " + motor.getStatorCurrent().getValueAsDouble());
        return Math.abs(motor.getStatorCurrent().getValueAsDouble()) >= Constants.IntakeExtension.STALLING_CURRENT;
    }

    /**
     * When in simulation, updates the ElevatorSim and
     * writes the new rotor position and velocity into the TalonFX sim state.
     */
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage elevatorMotorVoltage = simState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            sim.setInputVoltage(elevatorMotorVoltage.in(Volts));
            sim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(sim.getPositionMeters() * Constants.IntakeExtension.GEAR_RATIO);
            simState.setRotorVelocity(sim.getVelocityMetersPerSecond() * Constants.IntakeExtension.GEAR_RATIO);
        }
    }
    
    /**
     * Checks RobotContainer to see if
     * this subsystem is running in simulation mode.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_EXTENSION_INDEX) == SubsystemStatus.Simulated;
    }

    /**
     * Checks if the subsystem is marked disabled.
     * Blocks all motor commands when it is.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_EXTENSION_INDEX) == SubsystemStatus.Disabled;
    }

    /*
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("IntakeExtension")
                .voltage(getVoltage())
                .angularPosition(motor.getPosition().getValue())
                .angularVelocity(getVelocity()),
        this)
    );

    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
        */

    public static IntakeExtension getInstance()
    {
        if(instance == null) instance = new IntakeExtension();
        return instance;
    }
}
