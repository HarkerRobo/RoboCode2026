package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class Climb extends SubsystemBase {
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

    private Climb() 
    {
        config();
        
        if (Robot.isSimulation())
        {
            TalonFXSimState elevatorSimState = elevator.getSimState();
            elevatorSimState.Orientation = Constants.Climb.ELEVATOR_MECHANICAL_ORIENTATION;
            elevatorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            TalonFXSimState climbSimState = climb.getSimState();
            climbSimState.Orientation = Constants.Climb.CLIMB_MECHANICAL_ORIENTATION;
            climbSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    //configurates the subsystem
    private void config() 
    {
        targetPosition = Rotations.of(0);
        elevator = new TalonFX(Constants.Climb.ELEVATOR_ID);
        climb = new TalonFX(Constants.Climb.HINGE_ID);

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

    public void setElevatorDutyCycle(double velocity) 
    {
        elevator.setControl(new DutyCycleOut(velocity));
    }

    public void setElevatorVelocity(AngularVelocity velocity) 
    {
        elevator.setControl(new VelocityVoltage(velocity));
    }

    public AngularVelocity getElevatorVelocity() 
    {
        return elevator.getVelocity().getValue();
    }

    public Angle getElevatorPosition()
    {
        return elevator.getPosition().getValue();
    }

    public Angle getElevatorTargetPosition() 
    {
        return targetPosition;
    }

    public Voltage getElevatorVoltage()
    {
        return elevator.getMotorVoltage().getValue();
    }

    public void setElevatorVoltage(Voltage v)
    {
        elevator.setControl(new VoltageOut(v));
    }

    public void setElevatorTargetPosition(Angle tPosition) 
    {
        targetPosition = tPosition;
        elevator.setControl(new MotionMagicVoltage(tPosition));
    }

    public boolean isElevatorStalling()
    {
        return elevator.getStatorCurrent().getValueAsDouble() >= Constants.Climb.ELEVATOR_STALLING_CURRENT.in(Amps);
    }

    public void setClimbDutyCycle(double velocity) 
    {
        climb.setControl(new DutyCycleOut(velocity));
    }

    public Voltage getClimbVoltage()
    {
        return climb.getMotorVoltage().getValue();
    }

    public void setClimbVoltage(Voltage v)
    {
        climb.setControl(new VoltageOut(v));
    }
    
    
    @Override
    public void simulationPeriodic()
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
        elevatorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * Constants.Climb.ELEVATOR_GEAR_RATIO);
        

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
        climbSimState.setRawRotorPosition(climbSim.getAngularPosition().in(Rotations) * Constants.Climb.CLIMB_GEAR_RATIO);
        climbSimState.setRotorVelocity(climbSim.getAngularVelocity().in(Rotations.per(Second)) * Constants.Climb.CLIMB_GEAR_RATIO);
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(0.1),Volts.of(0.2),Seconds.of(5.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->setElevatorVoltage(v),
            (SysIdRoutineLog l)->l
                .motor("Climb")
                .voltage(getElevatorVoltage())
                .angularPosition(elevator.getPosition().getValue())
                .angularVelocity(getElevatorVelocity()),
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