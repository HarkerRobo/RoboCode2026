package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * controls the balls from intake to indexer
 */
public class Hopper extends SubsystemBase
{
    private static Hopper instance;
    private TalonFX master;
    private double desiredPosition; // rotations

    private Hopper()
    {
        master = new TalonFX(Constants.Hopper.MOTOR_ID);
        master.clearStickyFaults();
        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.Feedback.SensorToMechanismRatio = Constants.Hopper.GEAR_RATIO;
        
        masterConfig.CurrentLimits.StatorCurrentLimit = Constants.Hopper.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Hopper.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Hopper.MM_CRUISE_VELOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Hopper.MM_ACCELERATION;
        masterConfig.MotionMagic.MotionMagicJerk = Constants.Hopper.MM_JERK;
        masterConfig.MotorOutput.Inverted = Constants.Hopper.INVERTED;

        masterConfig.Slot0.kP = Constants.Hopper.KP;
        masterConfig.Slot0.kI = Constants.Hopper.KI;
        masterConfig.Slot0.kD = Constants.Hopper.KD;

        masterConfig.Slot0.kG = Constants.Hopper.KS;
        masterConfig.Slot0.kG = Constants.Hopper.KG;
        masterConfig.Slot0.kV = Constants.Hopper.KV;
        masterConfig.Slot0.kV = Constants.Hopper.KA;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(masterConfig);
    }


    /**
     * @return rotations
     */
    public double getPosition() 
    {
        return master.getPosition().getValueAsDouble();
    }
    
    /**
     * @return rotations per second
     */
    public double getVelocity()
    {
        return master.getVelocity().getValueAsDouble();
    }

    /**
     * @return voltage
     */
    public double getVoltage()
    {
        return master.getMotorVoltage().getValueAsDouble();
    }

    public double getDesiredPosition()
    {
        return this.desiredPosition;
    }

    public void setVelocity(double velocity)
    {
        master.setControl(new VelocityVoltage(velocity));
    }

    public void setPosition(double position)
    {
        master.setPosition(position);
    }

    public boolean isStalling()
    {
        return master.getStatorCurrent().getValueAsDouble() >= Constants.Hopper.HOPPER_STALLING_CURRENT;
    }

    public boolean atPosition()
    {
        return Math.abs(this.desiredPosition - getPosition()) <= Constants.EPSILON;
    }

    /**
     * @param power variable between [-1, 1]
     */
    public void setDutyCycle(double power)
    {
        master.setControl(new DutyCycleOut(power));
    }

    //sets the target & moves there
    public void setDesiredPosition(double target)
    {
        this.desiredPosition = target;
        master.setControl(new MotionMagicVoltage(target));
    }


    public static Hopper getInstance() 
    {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }
}
