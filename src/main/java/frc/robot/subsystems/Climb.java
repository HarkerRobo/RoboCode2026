package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private Angle targetPosition;

    private TalonFX elevator;
    private TalonFX climb;

    private Climb() 
    {
        config();
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