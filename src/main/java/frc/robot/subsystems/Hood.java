package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX motor;

    private Angle desiredPosition = Degrees.of(65); // degrees
    
    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.001, Constants.Hood.GEAR_RATIO),
        DCMotor.getKrakenX44(1));

    private Debouncer debouncer = new Debouncer(Constants.Hood.DEBOUNCE_TIME.in(Seconds));

    /**
     * Initializes the master/follower TalonFX motors and applies configuration.
     * If simulated, sets up motor orientation and initializes the stall simulator.
    */
    private Hood()
    {
        motor = new TalonFX(Constants.Hood.ID, Constants.CAN_SUPERSTRUCTURE);

        config();

        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();
            simState.Orientation = Constants.Hood.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }
    }


    /**
     * Applies all TalonFX configuration including PID, feedforward, inversion, limits, and sensor ratio.
     * Ensures the hood motor starts in fully configured state.
     */
    private void config()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (!isSimulated())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_CURRENT_LIMIT.in(Amps);
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT.in(Amps);
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Hood.GEAR_RATIO;

        config.MotorOutput.Inverted = Constants.Hood.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        
        config.Slot0.kS = Constants.Hood.KS;
        config.Slot0.kV = Constants.Hood.KV;
        config.Slot0.kA = Constants.Hood.KA;
        config.Slot0.kG = Constants.Hood.KG;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE.in(Volts);
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE.in(Volts);

        motor.getConfigurator().apply(config);
    }

    /**
    * Tells the hood to head toward a specific angle using Motion Magic.
    * Also remembers that angle so other code can check where it was told to go.
    */
    public void moveToPosition(Angle desiredPosition)
    {
        this.desiredPosition = desiredPosition;
        motor.setControl(new PositionVoltage(desiredPosition));
    }
    
    /**
    * Grabs the hood’s current angle straight from the motor sensor.
    * Good for anything that needs to know where the hood actually is.
    */
    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }
    
    /**
    * Returns the voltage currently applied to the hood motor.
    * Useful for getting the voltage.
    */
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
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
        motor.setPosition(position, 0.5);
    }

    /**
     * Applies a direct voltage to the hood motor.
     * Blocks the command when the subsystem is disabled.
     */
    public void setVoltage(Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        motor.setVoltage(voltage.in(Volts));
    }

    /**
    * Checks if the hood is at the angle it was told to reach.
    * Returns true when it’s there.
    */
    public boolean readyToShoot ()
    {
        return Math.abs(motor.getPosition().getValue().minus(desiredPosition).in(Rotations)) < (Constants.Hood.MAX_ERROR).in(Rotations);
    }
    
    /**
    * Returns the last commanded hood angle.
    * See where what its aiming for
    */
    public Angle getDesiredPosition()
    {
        return desiredPosition;
    }
    
    /**
    * Detects whether the hood is drawing stall‑level current.
    * In simulation, also checks the StallSimulator for virtual stalls.
    */
    public boolean isStalling()
    {
        return debouncer.calculate(Math.abs(motor.getStatorCurrent().getValueAsDouble()) >= Constants.Hood.STALLING_CURRENT.in(Amps));
    }

    /**
     * Returns the stator current drawn by the hood motor.
     */
    public Current getStatorCurrent()
    {
        return Amps.of(motor.getStatorCurrent().getValueAsDouble());
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
            simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Hood.GEAR_RATIO));
            simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Hood.GEAR_RATIO));
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
     * Returns true if the subsystem is disabled.
     * Prevents all motor commands from being applied.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOOD_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1.0).div(Seconds.of(1.)),Volts.of(5.0),Seconds.of(5.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Hood")
                .voltage(getVoltage())
                .angularPosition(getPosition())
                .angularVelocity(motor.getVelocity().getValue()),
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
        return sysId.dynamic(direction).withName("SysId D" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
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
