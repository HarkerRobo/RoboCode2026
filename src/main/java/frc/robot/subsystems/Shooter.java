
package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(1));

    private Shooter()
    {
        motor = new TalonFX(Constants.Shooter.MOTOR_ID);

        config();
        
        if (Robot.isSimulation())
        {
            motor.getSimState().Orientation = Constants.Shooter.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        
    }

    private void config()
    {
        motor.clearStickyFaults();
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
        config.Slot0.kS = Constants.Shooter.KS;
        config.Slot0.kG = Constants.Shooter.KG;
        config.Slot0.kV = Constants.Shooter.KV;
        config.Slot0.kA = Constants.Shooter.KA;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);
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

    public void setVelocity (AngularVelocity velocity)
    {
        motor.setControl(new VelocityVoltage(velocity));
    }

    public void setVoltage (Voltage voltage)
    {
        motor.setControl(new VoltageOut(voltage));
    }

    public void setDutyCycle (double dutyCycle)
    {
        motor.setControl(new DutyCycleOut(dutyCycle));
    }
    
    @Override
    public void simulationPeriodic ()
    {
        TalonFXSimState simState = motor.getSimState();

        // set the supply voltage of the TalonFX
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        sim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
        sim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
        simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
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
    
    public static Shooter getInstance()
    {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
