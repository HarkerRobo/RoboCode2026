
package frc.robot.subsystems;

import java.nio.file.attribute.PosixFileAttributeView;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * 
 */
public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX motor;

    private double desiredPosition; // rotations

    private Hood()
    {
        motor = new TalonFX(Constants.Hood.MOTOR_ID);

        config();
    }

    private void config()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (Robot.isReal())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Hood.GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Hood.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Hood.MM_JERK;

        config.MotorOutput.Inverted = Constants.Hood.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        config.Slot0.kG = Constants.Hood.KG;
        config.Slot0.kV = Constants.Hood.KV;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);
    }

    public void moveToPosition(Angle desiredPosition)
    {
        this.desiredPosition = desiredPosition.in(Rotations);
        motor.setControl(new MotionMagicVoltage(desiredPosition));
    }

    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }
    
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }
    
    public boolean readyToShoot ()
    {
        return Math.abs(motor.getPosition().getValue().in(Rotation) - desiredPosition) < Constants.EPSILON;
    }
    
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }
    

    public static Hood getInstance()
    {
        if (instance == null) instance = new Hood();
        return instance;
    }
}
