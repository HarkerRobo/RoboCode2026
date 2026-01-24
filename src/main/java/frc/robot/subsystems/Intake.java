package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
   private static Intake instance;
   private static TalonFX motor;
   
   private Intake()
   {
        motor = new TalonFX(Constants.Intake.MOTOR_ID);
        config();
   }

   private void config()
   {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = Constants.Intake.GEAR_RATIO;

        config.MotorOutput.Inverted = Constants.Intake.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        config.Slot0.kG = Constants.Hood.KG;
        config.Slot0.kV = Constants.Hood.KV;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        config.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
   }
   
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }
   
    public void setVoltage (Voltage voltage)
    {
        motor.setControl(new VoltageOut(voltage));
    }

    public void setVelocity (double velocity)
    {
        motor.setControl(new VelocityVoltage(velocity));
    }

    public void setDutyCycle(double velocity) {
        motor.setControl(new DutyCycleOut(velocity));
    }

   public static Intake getInstance()
   {
        if(instance == null) instance = new Intake();
        return instance;
   }
}
