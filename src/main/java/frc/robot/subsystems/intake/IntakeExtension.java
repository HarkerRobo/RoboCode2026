package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeExtension extends SubsystemBase 
{
    private static IntakeExtension instance;
    private TalonFX extension;

    private IntakeExtension()
    {
        extension = new TalonFX(Constants.IntakeExtension.ID);
        config();
        
        if (Robot.isSimulation())
        {
            extension.getSimState().Orientation = Constants.IntakeExtension.MECHANICAL_ORIENTATION;
            extension.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        extension.clearStickyFaults();
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        extensionConfig.Feedback.SensorToMechanismRatio = Constants.IntakeExtension.GEAR_RATIO;

        extensionConfig.MotorOutput.Inverted = Constants.IntakeExtension.INVERTED;
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        extensionConfig.Slot0.kP = Constants.IntakeExtension.KP;
        extensionConfig.Slot0.kI = Constants.IntakeExtension.KI;
        extensionConfig.Slot0.kD = Constants.IntakeExtension.KD;
        extensionConfig.Slot0.kS = Constants.IntakeExtension.KS;
        extensionConfig.Slot0.kV = Constants.IntakeExtension.KV;
        extensionConfig.Slot0.kA = Constants.IntakeExtension.KA;

        extensionConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        extensionConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.STATOR_CURRENT_LIMIT;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakeExtension.SUPPLY_CURRENT_LIMIT;

        extension.getConfigurator().apply(extensionConfig);
    }
    
    public Voltage getExtensionVoltage()
    {
        return extension.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getExtensionVelocity()
    {
        return extension.getVelocity().getValue();
    }
   
    public void setExtensionVoltage (Voltage voltage)
    {
        extension.setControl(new VoltageOut(voltage));
    }

    public void setExtensionVelocity (AngularVelocity velocity)
    {
        //System.out.println("Velocity set to " + velocity);
        extension.setControl(new VelocityVoltage(velocity));
    }

    public void setExtensionDutyCycle(double velocity) 
    {
        extension.setControl(new DutyCycleOut(velocity));
    }

    public boolean extensionIsStalling()
    {
        return Math.abs(extension.getStatorCurrent().getValueAsDouble()) >= Constants.IntakeExtension.STALLING_CURRENT;
    }
    
    @Override
    public void simulationPeriodic ()
    {
    }
    
    /*
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Intake")
                .voltage(getVoltage())
                .angularPosition(motor.getPosition().getValue())
                .angularVelocity(getVelocity()),
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
        */

    public static IntakeExtension getInstance()
    {
        if(instance == null) instance = new IntakeExtension();
        return instance;
    }
}
