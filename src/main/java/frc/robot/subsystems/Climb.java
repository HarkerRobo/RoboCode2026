package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Climb extends SubsystemBase 
{
    private static Climb instance;

    private TalonFX climbWheels;
    private TalonFX spooling;

    // this doesn't work anymore since its not an elevator and idk how you would sim it
    private DCMotorSim spoolSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Climb.SPOOLING_GEAR_RATIO),
            DCMotor.getKrakenX60(1));


    private DCMotorSim climbWheelsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0.001, Constants.Climb.CLIMBWHEELS_GEAR_RATIO),
        DCMotor.getKrakenX60(1));

    /**
     * Initializes the elevator + climb motors, simulation models, and default target position.
     * Also applies mechanical orientation and motor type when running in simulation.
     */
    private Climb() 
    {
        climbWheels = new TalonFX(Constants.Climb.CLIMBWHEELS_ID, Constants.CAN_SUPERSTRUCTURE);
        spooling = new TalonFX(Constants.Climb.SPOOLING_ID, Constants.CAN_SUPERSTRUCTURE);

        config();
        
        if (isSimulated())
        {
            TalonFXSimState elevatorSimState = climbWheels.getSimState();
            elevatorSimState.Orientation = Constants.Climb.CLIMBWHEELS_MECHANICAL_ORIENTATION;
            elevatorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            TalonFXSimState climbSimState = spooling.getSimState();
            climbSimState.Orientation = Constants.Climb.SPOOLING_MECHANICAL_ORIENTATION;
            climbSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX configs (PID, current limits, inversion, Motion Magic, etc.) to both motor
     * Makes the subsystem boots into a fully‑configured state.
     */
    
    private void config() 
    {
        climbWheels.clearStickyFaults();
        spooling.clearStickyFaults();

        TalonFXConfiguration climbWheelsConfig = new TalonFXConfiguration();

        climbWheelsConfig.MotorOutput.Inverted = Constants.Climb.CLIMBWHEELS_INVERTED;

        climbWheelsConfig.Feedback.SensorToMechanismRatio = Constants.Climb.CLIMBWHEELS_GEAR_RATIO;
        
        climbWheelsConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.CLIMBWHEELS_STATOR_CURRENT_LIMIT.in(Amps);
        climbWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climbWheelsConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.CLIMBWHEELS_SUPPLY_CURRENT_LIMIT.in(Amps);
        climbWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climbWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbWheelsConfig.Slot0.kP = Constants.Climb.KP_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kI = Constants.Climb.KI_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kD = Constants.Climb.KD_CLIMBWHEELS;

        climbWheels.getConfigurator().apply(climbWheelsConfig);

        TalonFXConfiguration spoolingConfig = new TalonFXConfiguration();

        spoolingConfig.MotorOutput.Inverted = Constants.Climb.SPOOLING_INVERTED;

        spoolingConfig.Feedback.SensorToMechanismRatio = Constants.Climb.SPOOLING_GEAR_RATIO;
        
        spoolingConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.SPOOLING_STATOR_CURRENT_LIMIT.in(Amps);
        spoolingConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        spoolingConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SPOOLING_SUPPLY_CURRENT_LIMIT.in(Amps);
        spoolingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        spoolingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        spoolingConfig.Slot0.kP = Constants.Climb.KP_SPOOLING;
        spoolingConfig.Slot0.kI = Constants.Climb.KI_SPOOLING;
        spoolingConfig.Slot0.kD = Constants.Climb.KD_SPOOLING;

        spooling.getConfigurator().apply(spoolingConfig);
    }

    /**
     * Directly drives the elevator motor with a raw duty cycle percentage.
     * Disabled when the subsystem is marked “Disabled”.
    */
    public void setElevatorDutyCycle(double velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Returns the current climb wheels velocity.
     */
    public AngularVelocity getClimbWheelsVelocity() 
    {
        return climbWheels.getVelocity().getValue();
    }

    /**
     * Returns the voltage applied to the climb wheels motor.
     */
    public Voltage getClimbWheelsVoltage()
    {
        return climbWheels.getMotorVoltage().getValue();
    }

    /**
     * Returns the climb wheels position in rotations.
     */
    public Angle getClimbWheelsPosition()
    {
        return climbWheels.getPosition().getValue();
    }

    /**
     * Applies a direct voltage to the climb wheels motor.
     * Blocks the command if the subsystem is disabled.
     */
    public void setClimbWheelsVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VoltageOut(v));
    }

    /**
     * Returns true if the spooling motor is stalling.
     * Uses stator current to detect mechanical resistance.
     */
    public boolean isSpoolingStalling()
    {
        return Math.abs(spooling.getStatorCurrent().getValueAsDouble()) >= Constants.Climb.SPOOLING_STALLING_CURRENT.in(Amps);
    }

    /**
     * Returns the voltage applied to the spooling motor.
     */
    public Voltage getSpoolingVoltage()
    {
        return spooling.getMotorVoltage().getValue();
    }
    
    /**
     * Returns the spooling motor position in rotations.
     * Used for tracking rope payout or encoder diagnostics.
     */
    public Angle getSpoolingPosition()
    {
        return spooling.getPosition().getValue();
    }

    /**
     * Applies a direct voltage to the spooling motor.
     * Blocks the command if the subsystem is disabled.
     */
    public void setSpoolingVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        spooling.setControl(new VoltageOut(v));
    }
        /**
     * When in simulation, updates both elevator and climb physics.
     * Feeds simulated rotor position/velocity back into the TalonFXSimState.
     */
    @Override
    public void periodic()
    {
        if (isSimulated())
        {
            TalonFXSimState climbWheelsSimState = climbWheels.getSimState();

            // set the supply voltage of the TalonFX
            climbWheelsSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage climbWheelsMotorVoltage = climbWheelsSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            climbWheelsSim.setInputVoltage(climbWheelsMotorVoltage.in(Volts));
            climbWheelsSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            climbWheelsSimState.setRawRotorPosition(climbWheelsSim.getAngularPositionRotations() * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);
            climbWheelsSimState
                    .setRotorVelocity(climbWheelsSim.getAngularVelocity().in(RotationsPerSecond) * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);


            TalonFXSimState spoolingSimState = spooling.getSimState();

            // set the supply voltage of the TalonFX
            spoolingSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage climbMotorVoltage = spoolingSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            spoolSim.setInputVoltage(climbMotorVoltage.in(Volts));
            spoolSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            spoolingSimState.setRawRotorPosition(
                    spoolSim.getAngularPosition().in(Rotations) * Constants.Climb.SPOOLING_GEAR_RATIO);
            spoolingSimState.setRotorVelocity(
                    spoolSim.getAngularVelocity().in(Rotations.per(Second)) * Constants.Climb.SPOOLING_GEAR_RATIO);
        }
    }

    /**
     * Checks RobotContainer to see if
     * this subsystem is running in simulation mode.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Checks if the subsystem is marked disabled.
     * Blocks all motor commands when it is.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Disabled;
    }
    
    //returns the instance of the subsystem
    public static Climb getInstance() 
    {
        if (instance == null) 
        {
            instance = new Climb();
        }
        return instance;
    }
}