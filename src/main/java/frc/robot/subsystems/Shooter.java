
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

/**
 * This class controls the powering of the flywheels to propel the fuel
 */
public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    private TalonFX left;
    private TalonFX right;
    
    private DCMotorSim leftSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(2));
    
    private DCMotorSim rightSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(2));

    private double leftTargetVelocity = 0.0;
    private double rightTargetVelocity = 0.0;

    private Shooter()
    {
        left = new TalonFX(Constants.Shooter.LEFT_ID, Constants.CAN_SUPERSTRUCTURE);
        right = new TalonFX(Constants.Shooter.RIGHT_ID, Constants.CAN_SUPERSTRUCTURE);

        config();
        
        if (isSimulated())
        {
            left.getSimState().Orientation = Constants.Shooter.LEFT_MECHANICAL_ORIENTATION;
            left.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            right.getSimState().Orientation = Constants.Shooter.RIGHT_MECHANICAL_ORIENTATION;
            right.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        
    }

    
    private void config()
    {
        left.clearStickyFaults();
        right.clearStickyFaults();

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

        left.getConfigurator().apply(config);
        right.getConfigurator().apply(config);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.Slot0.kP = Constants.Shooter.LEFT_KP;
        leftConfig.Slot0.kI = Constants.Shooter.LEFT_KI;
        leftConfig.Slot0.kD = Constants.Shooter.LEFT_KD;
        leftConfig.Slot0.kS = Constants.Shooter.LEFT_KS;
        leftConfig.Slot0.kV = Constants.Shooter.LEFT_KV;
        leftConfig.Slot0.kA = Constants.Shooter.LEFT_KA;
        leftConfig.MotorOutput.Inverted = Constants.Shooter.LEFT_INVERTED;
        left.getConfigurator().apply(leftConfig);
        
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.Slot0.kP = Constants.Shooter.RIGHT_KP;
        rightConfig.Slot0.kI = Constants.Shooter.RIGHT_KI;
        rightConfig.Slot0.kD = Constants.Shooter.RIGHT_KD;
        rightConfig.Slot0.kS = Constants.Shooter.RIGHT_KS;
        rightConfig.Slot0.kV = Constants.Shooter.RIGHT_KV;
        rightConfig.Slot0.kA = Constants.Shooter.RIGHT_KA;
        leftConfig.MotorOutput.Inverted = Constants.Shooter.RIGHT_INVERTED;
        right.getConfigurator().apply(rightConfig);
    }
    
    /**
     * Returns the voltage applied to the left flywheel motor.
     */
    public Voltage getLeftVoltage()
    {
        return left.getMotorVoltage().getValue();
    }
    
    /**
     * Returns the voltage applied to the right flywheel motor.
     */
    public Voltage getRightVoltage()
    {
        return right.getMotorVoltage().getValue();
    }
    
    /**
     * Returns the angular velocity of the left flywheel.
     */
    public AngularVelocity getLeftVelocity()
    {
        return left.getVelocity().getValue();
    }
    
    /**
     * Returns the angular velocity of the right flywheel.
     */
    public AngularVelocity getRightVelocity()
    {
        return right.getVelocity().getValue();
    }

    /**
     * Returns the linear surface speed of the left flywheel.
     * Converts angular ro meters per second
     */
    public LinearVelocity getLeftEffectiveVelocity()
    {
        return MetersPerSecond.of(getLeftVelocity().in(RotationsPerSecond) * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }
    
    /**
     * Returns the linear surface speed of the right flywheel.
     * Converts angular velocity into meters per second.
     */
    public LinearVelocity getRightEffectiveVelocity()
    {
        return MetersPerSecond.of(getRightVelocity().in(RotationsPerSecond) * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }

    /**
     * Returns the last commanded angular velocity for the left flywheel.
     */
    public AngularVelocity getLeftTargetVelocity()
    {
        return RotationsPerSecond.of(leftTargetVelocity);
    }

    /**
     * Returns the target linear surface speed of the left flywheel.
     * Converts the stored angular target into meters per second.
     */
    public LinearVelocity getLeftEffectiveTargetVelocity()
    {
        return MetersPerSecond.of(leftTargetVelocity * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }
    
    /**
     * Returns the last commanded angular velocity for the right flywheel.
     * Useful for comparing target and measured speeds.
     */
    public AngularVelocity getRightTargetVelocity()
    {
        return RotationsPerSecond.of(rightTargetVelocity);
    }

    /**
     * Returns the target linear surface speed of the right flywheel.
     * Converts the stored angular target into meters per second.
     */
    public LinearVelocity getRightEffectiveTargetVelocity()
    {
        return MetersPerSecond.of(rightTargetVelocity * Constants.Shooter.FLYWHEEL_CIRCUMFERANCE);
    }

    /**
     * Sets both flywheels to the same angular velocity.
     * Calls the individual left and right setters internally.
     */
    public void setVelocity (AngularVelocity velocity)
    {
        setLeftVelocity(velocity);
        setRightVelocity(velocity);
    }

    /**
     * Commands the left flywheel to a target angular velocity.
     * Blocks the command when the subsystem is disabled.
     */
    public void setLeftVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        left.setControl(new VelocityVoltage(velocity));
        leftTargetVelocity = velocity.in(RotationsPerSecond);
    }

    /**
     * Sets the left flywheel using a linear surface speed.
     * Converts meters per second into angular velocity.
     */
    public void setLeftEffectiveVelocity(LinearVelocity velocity)
    {
        setLeftVelocity(RotationsPerSecond.of(velocity.in(MetersPerSecond) / Constants.Shooter.FLYWHEEL_CIRCUMFERANCE));
    }

    /**
     * Commands the right flywheel to a target angular velocity.
     * Blocks the command when the subsystem is disabled.
     */
    public void setRightVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        right.setControl(new VelocityVoltage(velocity));
        rightTargetVelocity = velocity.in(RotationsPerSecond);
    }

    /**
     * Sets the right flywheel using a linear surface speed.
     * Converts meters per second into angular velocity.
     */
    public void setRightEffectiveVelocity(LinearVelocity velocity)
    {
        setRightVelocity(RotationsPerSecond.of(velocity.in(MetersPerSecond) / Constants.Shooter.FLYWHEEL_CIRCUMFERANCE));
    }
    
    /**
     * Sets both flywheels using a linear surface speed.
     * Converts meters per second into angular velocity for each side.
     */
    public void setEffectiveVelocity (LinearVelocity velocity)
    {
        setLeftEffectiveVelocity(velocity);
        setRightEffectiveVelocity(velocity);
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
        left.setControl(new VoltageOut(voltage));
        right.setControl(new VoltageOut(voltage));
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
        left.setControl(new DutyCycleOut(dutyCycle));
        right.setControl(new DutyCycleOut(dutyCycle));
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
            TalonFXSimState leftSimState = left.getSimState();

            // set the supply voltage of the TalonFX
            leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            leftSim.setInputVoltage(leftSimState.getMotorVoltageMeasure().in(Volts));
            leftSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            leftSimState.setRawRotorPosition(leftSim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
            leftSimState.setRotorVelocity(leftSim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));

            TalonFXSimState rightSimState = right.getSimState();

            // set the supply voltage of the TalonFX
            rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            rightSim.setInputVoltage(rightSimState.getMotorVoltageMeasure().in(Volts));
            rightSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            rightSimState.setRawRotorPosition(rightSim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
            rightSimState.setRotorVelocity(rightSim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));
        }
    }
    
    private SysIdRoutine leftSysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->left.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
                .voltage(getLeftVoltage())
                .angularPosition(left.getPosition().getValue())
                .angularVelocity(getLeftVelocity()),
        this)
    );
    

    /**
     * Creates a quasistatic SysId test for the left flywheel.
     * Used to characterize feedforward constants.
     */
    public boolean readyToShoot()
    {
        return Math.abs(left.getVelocity().getValue().in(Rotations.per(Second)) - leftTargetVelocity) < 0.5 ||
               Math.abs(right.getVelocity().getValue().in(Rotations.per(Second)) - rightTargetVelocity) < 0.5;
    }
    
    /**
     * Creates a quasistatic SysId test for the left flywheel.
     * Used to characterize feedforward constants.
     */
    public Command leftSysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return leftSysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    /**
     * Creates a dynamic SysId test for the left flywheel.
     * Measures acceleration response for tuning.
     */

    public Command leftSysIdDynamic (SysIdRoutine.Direction direction)
    {
        return leftSysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }

    
    private SysIdRoutine rightSysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->right.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
                .voltage(getRightVoltage())
                .angularPosition(right.getPosition().getValue())
                .angularVelocity(getRightVelocity()),
        this)
    );

    
    /**
     * Creates a quasistatic SysId test for the right flywheel.
     * Used to characterize feedforward constants.
     */
    public Command rightSysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return rightSysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    

    /**
     * Creates a dynamic SysId test for the right flywheel.
     * Measures acceleration response for tuning.
     */
    public Command rightSysIdDynamic (SysIdRoutine.Direction direction)
    {
        return rightSysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
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
