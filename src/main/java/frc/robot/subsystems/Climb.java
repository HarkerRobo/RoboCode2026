package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class Climb extends SubsystemBase 
{
    private static Climb instance;

    private Angle targetPosition;

    private TalonFX elevator;
    private TalonFX climb;

    private ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Climb.ELEVATOR_GEAR_RATIO),
            DCMotor.getKrakenX60(1), Constants.Climb.ELEVATOR_MIN_HEIGHT, Constants.Climb.ELEVATOR_MAX_HEIGHT, true, Constants.Climb.ELEVATOR_MIN_HEIGHT);


    private DCMotorSim climbSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0.001, Constants.Climb.CLIMB_GEAR_RATIO),
        DCMotor.getKrakenX60(1));

    /**
     * Initializes the elevator + climb motors, simulation models, and default target position.
     * Also applies mechanical orientation and motor type when running in simulation.
     */
    private Climb() 
    {
        targetPosition = Rotations.of(0);
        elevator = new TalonFX(Constants.Climb.ELEVATOR_ID);
        climb = new TalonFX(Constants.Climb.HINGE_ID);

        config();
        
        if (isSimulated())
        {
            TalonFXSimState elevatorSimState = elevator.getSimState();
            elevatorSimState.Orientation = Constants.Climb.ELEVATOR_MECHANICAL_ORIENTATION;
            elevatorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            TalonFXSimState climbSimState = climb.getSimState();
            climbSimState.Orientation = Constants.Climb.CLIMB_MECHANICAL_ORIENTATION;
            climbSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX configs (PID, current limits, inversion, Motion Magic, etc.) to both motor
     * Makes the subsystem boots into a fully‑configured state.
     */
    
    private void config() 
    {

        elevator.clearStickyFaults();
        climb.clearStickyFaults();

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.Inverted = Constants.Climb.ELEVATOR_INVERTED;

        elevatorConfig.Feedback.SensorToMechanismRatio = Constants.Climb.ELEVATOR_GEAR_RATIO;
        
        elevatorConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.STATOR_CURRENT_LIMIT.in(Amps);
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SUPPLY_CURRENT_LIMIT.in(Amps);
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorConfig.Slot0.kP = Constants.Climb.KP_ELEVATOR;
        elevatorConfig.Slot0.kI = Constants.Climb.KI_ELEVATOR;
        elevatorConfig.Slot0.kD = Constants.Climb.KD_ELEVATOR;

        elevatorConfig.Slot0.kG = Constants.Climb.KG;
        elevatorConfig.Slot0.kS = Constants.Climb.KS;
        elevatorConfig.Slot0.kV = Constants.Climb.KV;
        elevatorConfig.Slot0.kA = Constants.Climb.KA;

        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.MM_CRUISE_VELOCITY;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = Constants.Climb.MM_ACCELERATION;
        elevatorConfig.MotionMagic.MotionMagicJerk = Constants.Climb.MM_JERK;

        elevator.getConfigurator().apply(elevatorConfig);

        TalonFXConfiguration climbConfig = new TalonFXConfiguration();

        climbConfig.MotorOutput.Inverted = Constants.Climb.CLIMB_INVERTED;

        climbConfig.Feedback.SensorToMechanismRatio = Constants.Climb.CLIMB_GEAR_RATIO;
        
        climbConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.STATOR_CURRENT_LIMIT.in(Amps);
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climbConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SUPPLY_CURRENT_LIMIT.in(Amps);
        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbConfig.Slot0.kP = Constants.Climb.KP_CLIMB;
        climbConfig.Slot0.kI = Constants.Climb.KI_CLIMB;
        climbConfig.Slot0.kD = Constants.Climb.KD_CLIMB;

        climb.getConfigurator().apply(climbConfig);
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
        elevator.setControl(new DutyCycleOut(velocity));
    }

    /**
    * Commands the elevator to hold a target velocity.
    * Automatically handles feedforward and closed‑loop control.
    */
    public void setElevatorVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        elevator.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Returns the current measured elevator angular velocity.
     * Useful for telemetry, tuning, or SysId logging .
     */
    public AngularVelocity getElevatorVelocity() 
    {
        return elevator.getVelocity().getValue();
    }

    /**
     * Returns the current elevator position in rotations.
     * Used for Motion Magic, soft limits, and state estimation.
     */
    public Angle getElevatorPosition()
    {
        return elevator.getPosition().getValue();
    }

    /**
     * Returns the last commanded Motion Magic target position.
     * Allows commands/telemetry to know where the elevator is trying to go.
     */
    public Angle getElevatorTargetPosition() 
    {
        return targetPosition;
    }

    /**
     * Returns the motor voltage being applied to the elevator.
     * Useful for SysId, debugging, and simulation.
     */
    public Voltage getElevatorVoltage()
    {
        return elevator.getMotorVoltage().getValue();
    }

    /**
     * Applies a direct voltage command to the elevator motor.
     * Used for SysId routines and physics‑accurate simulation.
     */
    public void setElevatorVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        elevator.setControl(new VoltageOut(v));
    }

    /**
     * Commands the elevator to move to a position using Motion Magic.
     * Stores the target and sends a MotionMagicVoltage request to the TalonFX.
     */
    public void setElevatorTargetPosition(Angle tPosition) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }

        targetPosition = tPosition;
        //System.out.println("Aiming to " + tPosition.in(Rotations) + ".");
        elevator.setControl(new MotionMagicVoltage(tPosition));
    }

    /**
     * Checks if the elevator stator current exceeds the stall threshold.
     * Useful for detecting jams probably maybe.
     */
    public boolean isElevatorStalling()
    {
        return Math.abs(elevator.getStatorCurrent().getValueAsDouble()) >= Constants.Climb.ELEVATOR_STALLING_CURRENT.in(Amps);
    }

    /**
     * Drives the climb hinge motor with a duty cycle.
     * Used for manual control
     */
    public void setClimbDutyCycle(double velocity) 
    {
        climb.setControl(new DutyCycleOut(velocity));
    }

    /**
     * Returns the voltage currently applied to the climb motor.
     * Useful for telemetry and SysId.
     */
    public Voltage getClimbVoltage()
    {
        return climb.getMotorVoltage().getValue();
    }

    /**
     * Applies a direct voltage to the climb motor
     * Used for SysId or simulation‑accurate testing. 
     */
    public void setClimbVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climb.setControl(new VoltageOut(v));
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
            TalonFXSimState elevatorSimState = elevator.getSimState();

            // set the supply voltage of the TalonFX
            elevatorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage elevatorMotorVoltage = elevatorSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            elevatorSim.setInputVoltage(elevatorMotorVoltage.in(Volts));
            elevatorSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            elevatorSimState.setRawRotorPosition(elevatorSim.getPositionMeters() * Constants.Climb.ELEVATOR_GEAR_RATIO);
            elevatorSimState
                    .setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * Constants.Climb.ELEVATOR_GEAR_RATIO);

            TalonFXSimState climbSimState = climb.getSimState();

            // set the supply voltage of the TalonFX
            climbSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage climbMotorVoltage = climbSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            climbSim.setInputVoltage(climbMotorVoltage.in(Volts));
            climbSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            climbSimState.setRawRotorPosition(
                    climbSim.getAngularPosition().in(Rotations) * Constants.Climb.CLIMB_GEAR_RATIO);
            climbSimState.setRotorVelocity(
                    climbSim.getAngularVelocity().in(Rotations.per(Second)) * Constants.Climb.CLIMB_GEAR_RATIO);
        }
    }

    /**
     * Returns true if the subsystem is marked as simulated in RobotContainer.
     * Used to decide whether to run physics simulation.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Returns true if the subsystem is disabled in RobotContainer.
     * Prevents motors from moving
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(0.1),Volts.of(0.5),Seconds.of(10.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->setElevatorVoltage(v),
            (SysIdRoutineLog l)->l
                .motor("Climb")
                .voltage(getElevatorVoltage())
                .angularPosition(elevator.getPosition().getValue())
                .angularVelocity(getElevatorVelocity()),
        this)
    );

    /**
     * Runs a SysId test on the elevator to characterize (or something like that) kS and kV.
     * Returns a command that logs voltage, position, and velocity.
     */
    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
     * Runs a SysId test to characterize acceleration‑related constants.
     * logs elevator motion for system identification.
     */
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
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