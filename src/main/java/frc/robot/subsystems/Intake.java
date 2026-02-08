package frc.robot.subsystems;
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

public class Intake extends SubsystemBase 
{
    private static Intake instance;
    private TalonFX main;
    private TalonFX extension;

    private DCMotorSim mainSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Intake.MAIN_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
   
    private Intake()
    {
        main = new TalonFX(Constants.Intake.MAIN_ID);
        extension = new TalonFX(Constants.Intake.EXTENSION_ID);
        config();
        
        if (Robot.isSimulation())
        {
            main.getSimState().Orientation = Constants.Intake.MAIN_MECHANICAL_ORIENTATION;
            main.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            extension.getSimState().Orientation = Constants.Intake.EXTENSION_MECHANICAL_ORIENTATION;
            extension.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        main.clearStickyFaults();
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();

        mainConfig.Feedback.SensorToMechanismRatio = Constants.Intake.MAIN_GEAR_RATIO;

        mainConfig.MotorOutput.Inverted = Constants.Intake.MAIN_INVERTED;
        mainConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        mainConfig.Slot0.kP = Constants.Intake.MAIN_KP;
        mainConfig.Slot0.kI = Constants.Intake.MAIN_KI;
        mainConfig.Slot0.kD = Constants.Intake.MAIN_KD;
        mainConfig.Slot0.kS = Constants.Intake.MAIN_KS;
        mainConfig.Slot0.kV = Constants.Intake.MAIN_KV;
        mainConfig.Slot0.kA = Constants.Intake.MAIN_KA;

        mainConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        mainConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        mainConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.MAIN_STATOR_CURRENT_LIMIT;
        mainConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.MAIN_SUPPLY_CURRENT_LIMIT;

        main.getConfigurator().apply(mainConfig);
        
        extension.clearStickyFaults();
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        extensionConfig.Feedback.SensorToMechanismRatio = Constants.Intake.EXTENSION_GEAR_RATIO;

        extensionConfig.MotorOutput.Inverted = Constants.Intake.EXTENSION_INVERTED;
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        extensionConfig.Slot0.kP = Constants.Intake.EXTENSION_KP;
        extensionConfig.Slot0.kI = Constants.Intake.EXTENSION_KI;
        extensionConfig.Slot0.kD = Constants.Intake.EXTENSION_KD;
        extensionConfig.Slot0.kS = Constants.Intake.EXTENSION_KS;
        extensionConfig.Slot0.kV = Constants.Intake.EXTENSION_KV;
        extensionConfig.Slot0.kA = Constants.Intake.EXTENSION_KA;

        extensionConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        extensionConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.EXTENSION_STATOR_CURRENT_LIMIT;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.EXTENSION_SUPPLY_CURRENT_LIMIT;

        extension.getConfigurator().apply(extensionConfig);
    }
   
    public Voltage getMainVoltage()
    {
        return main.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getMainVelocity()
    {
        return main.getVelocity().getValue();
    }
   
    public void setMainVoltage (Voltage voltage)
    {
        main.setControl(new VoltageOut(voltage));
    }

    public void setMainVelocity (AngularVelocity velocity)
    {
        System.out.println("Velocity set to " + velocity);
        main.setControl(new VelocityVoltage(velocity));
    }

    public void setMainDutyCycle(double velocity) 
    {
        main.setControl(new DutyCycleOut(velocity));
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
        System.out.println("Velocity set to " + velocity);
        extension.setControl(new VelocityVoltage(velocity));
    }

    public void setExtensionDutyCycle(double velocity) 
    {
        extension.setControl(new DutyCycleOut(velocity));
    }

    public boolean extensionIsStalling()
    {
        return extension.getSupplyCurrent().getValueAsDouble() >= Constants.Intake.INTAKE_STALLING_CURRENT;
    }
    
    @Override
    public void simulationPeriodic ()
    {
        TalonFXSimState simState = main.getSimState();

        // set the supply voltage of the TalonFX
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        mainSim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
        mainSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        simState.setRawRotorPosition(mainSim.getAngularPosition().times(Constants.Intake.MAIN_GEAR_RATIO));
        simState.setRotorVelocity(mainSim.getAngularVelocity().times(Constants.Intake.MAIN_GEAR_RATIO));
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

    public static Intake getInstance()
    {
        if(instance == null) instance = new Intake();
        return instance;
    }
}
