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

    private TalonFX climbWheels;
    private TalonFX spooling;

    private ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Climb.CLIMBWHEELS_GEAR_RATIO),
            DCMotor.getKrakenX60(1), Constants.Climb.CLIMBWHEELS_MIN_HEIGHT, Constants.Climb.CLIMBWHEELS_MAX_HEIGHT, true, Constants.Climb.CLIMBWHEELS_MIN_HEIGHT);


    private DCMotorSim climbSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0.001, Constants.Climb.SPOOLING_GEAR_RATIO),
        DCMotor.getKrakenX60(1));

    private Climb() 
    {
        targetPosition = Rotations.of(0);
        climbWheels = new TalonFX(Constants.Climb.CLIMBWHEELS_ID, Constants.CAN_CHAIN);
        spooling = new TalonFX(Constants.Climb.SPOOLING_ID, Constants.CAN_CHAIN);

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

    //configurates the subsystem
    private void config() 
    {

        climbWheels.clearStickyFaults();
        spooling.clearStickyFaults();

        TalonFXConfiguration climbWheelsConfig = new TalonFXConfiguration();

        climbWheelsConfig.MotorOutput.Inverted = Constants.Climb.CLIMBWHEELS_INVERTED;

        climbWheelsConfig.Feedback.SensorToMechanismRatio = Constants.Climb.CLIMBWHEELS_GEAR_RATIO;
        
        climbWheelsConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.STATOR_CURRENT_LIMIT.in(Amps);
        climbWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climbWheelsConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SUPPLY_CURRENT_LIMIT.in(Amps);
        climbWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climbWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbWheelsConfig.Slot0.kP = Constants.Climb.KP_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kI = Constants.Climb.KP_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kD = Constants.Climb.KP_CLIMBWHEELS;

        climbWheelsConfig.Slot0.kG = Constants.Climb.KG;
        climbWheelsConfig.Slot0.kS = Constants.Climb.KS;
        climbWheelsConfig.Slot0.kV = Constants.Climb.KV;
        climbWheelsConfig.Slot0.kA = Constants.Climb.KA;

        climbWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.MM_CRUISE_VELOCITY;
        climbWheelsConfig.MotionMagic.MotionMagicAcceleration = Constants.Climb.MM_ACCELERATION;
        climbWheelsConfig.MotionMagic.MotionMagicJerk = Constants.Climb.MM_JERK;

        climbWheels.getConfigurator().apply(climbWheelsConfig);

        TalonFXConfiguration spoolingConfig = new TalonFXConfiguration();

        spoolingConfig.MotorOutput.Inverted = Constants.Climb.SPOOLING_INVERTED;

        spoolingConfig.Feedback.SensorToMechanismRatio = Constants.Climb.SPOOLING_GEAR_RATIO;
        
        spoolingConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.STATOR_CURRENT_LIMIT.in(Amps);
        spoolingConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        spoolingConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SUPPLY_CURRENT_LIMIT.in(Amps);
        spoolingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        spoolingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        spoolingConfig.Slot0.kP = Constants.Climb.KP_SPOOLING;
        spoolingConfig.Slot0.kI = Constants.Climb.KI_SPOOLING;
        spoolingConfig.Slot0.kD = Constants.Climb.KD_SPOOLING;

        spooling.getConfigurator().apply(spoolingConfig);
    }

    public void setClimbWheelsDutyCycle(double velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new DutyCycleOut(velocity));
    }

    public void setClimbWheelsVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VelocityVoltage(velocity));
    }

    public AngularVelocity getClimbWheelsVelocity() 
    {
        return climbWheels.getVelocity().getValue();
    }

    public Angle getClimbWheelsPosition()
    {
        return climbWheels.getPosition().getValue();
    }

    public Angle getClimbWheelsTargetPosition() 
    {
        return targetPosition;
    }

    public Voltage getClimbWheelsVoltage()
    {
        return climbWheels.getMotorVoltage().getValue();
    }

    public void setClimbWheelsVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VoltageOut(v));
    }

    public void setClimbWheelsTargetPosition(Angle tPosition) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }

        targetPosition = tPosition;
        //System.out.println("Aiming to " + tPosition.in(Rotations) + ".");
        climbWheels.setControl(new MotionMagicVoltage(tPosition));
    }

    public boolean isSpoolingStalling()
    {
        return Math.abs(spooling.getStatorCurrent().getValueAsDouble()) >= Constants.Climb.SPOOLING_STALLING_CURRENT.in(Amps);
    }

    public void setSpoolingDutyCycle(double velocity) 
    {
        spooling.setControl(new DutyCycleOut(velocity));
    }

    public Voltage getSpoolingVoltage()
    {
        return spooling.getMotorVoltage().getValue();
    }

    public void setSpoolingVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        spooling.setControl(new VoltageOut(v));
    }
    
    
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
            elevatorSim.setInputVoltage(climbWheelsMotorVoltage.in(Volts));
            elevatorSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            climbWheelsSimState.setRawRotorPosition(elevatorSim.getPositionMeters() * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);
            climbWheelsSimState
                    .setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);

            TalonFXSimState climbSimState = spooling.getSimState();

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
                    climbSim.getAngularPosition().in(Rotations) * Constants.Climb.SPOOLING_GEAR_RATIO);
            climbSimState.setRotorVelocity(
                    climbSim.getAngularVelocity().in(Rotations.per(Second)) * Constants.Climb.SPOOLING_GEAR_RATIO);
        }
    }

    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(0.1),Volts.of(0.5),Seconds.of(10.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->setClimbWheelsVoltage(v),
            (SysIdRoutineLog l)->l
                .motor("Climb")
                .voltage(getClimbWheelsVoltage())
                .angularPosition(climbWheels.getPosition().getValue())
                .angularVelocity(getClimbWheelsVelocity()),
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