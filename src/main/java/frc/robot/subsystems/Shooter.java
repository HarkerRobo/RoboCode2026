
package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This class controls the powering of the flywheels to propel the fuel
 */
public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    private TalonFX motor;

    private Shooter()
    {
        motor = new TalonFX(Constants.Shooter.MOTOR_ID);

        config();
        
        setDefaultCommand(runOnce(()->setVelocity(RotationsPerSecond.of(Constants.Shooter.DEFAULT_VELOCITY))));
    }

    private void config()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (Robot.isReal())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Shooter.GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Shooter.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Shooter.MM_JERK;

        config.MotorOutput.Inverted = Constants.Shooter.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = Constants.Shooter.KP;
        config.Slot0.kI = Constants.Shooter.KI;
        config.Slot0.kD = Constants.Shooter.KD;
        config.Slot0.kG = Constants.Shooter.KG;
        config.Slot0.kV = Constants.Shooter.KV;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);
    }

    public void setVelocity (AngularVelocity velocity)
    {
        motor.setControl(new VelocityVoltage(velocity));
    }

    public void setVoltage (Voltage voltage)
    {
        motor.setControl(new VoltageOut(voltage));
    }

    public void setPosition (Angle position)
    {
        motor.setPosition(position);
    }
    
    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }
    
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    /**
     * 
     * @return rotations per second
     */
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }

    public boolean readyToShoot ()
    {
        return Math.abs(motor.getVelocity().getValue().in(RotationsPerSecond) - Constants.Shooter.SHOOT_VELOCITY) < Constants.EPSILON;
    }

    public static Shooter getInstance()
    {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}                 
