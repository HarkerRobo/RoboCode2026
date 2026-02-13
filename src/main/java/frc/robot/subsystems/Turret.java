package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Telemetry;

/**
 * for rotational (yaw) motion of the shooter.
 * This is coded for 360 degree motion.
 */
public class Turret extends SubsystemBase
{
    /*
    private static Turret instance;

    private TalonFX motor;

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Turret.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
 
    private double desiredPosition;

    private Turret ()
    {
        motor = new TalonFX(Constants.Turret.MOTOR_ID);


        config();

        if (Robot.isSimulation())
        {
            motor.getSimState().Orientation = Constants.Turret.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }

        motor.setPosition(Telemetry.getInstance().turretYawRawSubscriber.get());
        System.out.println("Read yaw position: " + Telemetry.getInstance().turretYawRawSubscriber.get());
    }

    private void config ()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Turret.GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Turret.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Turret.MM_JERK;

        config.MotorOutput.Inverted = Constants.Turret.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = Constants.Turret.KP;
        config.Slot0.kI = Constants.Turret.KI;
        config.Slot0.kD = Constants.Turret.KD;

        config.Slot0.kV = Constants.Turret.KV;
        config.Slot0.kG = Constants.Turret.KG;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.FORWARD_SOFTWARE_LIMIT_THRESHOLD;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.REVERSE_SOFTWARE_LIMIT_THRESHOLD;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);

    }

    @Override
    public void periodic ()
    {
        /*
        if (pitchMotor.getMotorVoltage().getValue().in(Volts) > 0 && pitchMotor.getPosition().getValue().in(Degrees) > Constants.Turret.MAX_PITCH)
        {
            System.out.println("Pitch motor position above " + Constants.Turret.MAX_PITCH + " causing pitch motor to be disabled");
            pitchMotor.disable();
        }

        if (pitchMotor.getMotorVoltage().getValue().in(Volts) < 0 && pitchMotor.getPosition().getValue().in(Degrees) < Constants.Turret.MIN_PITCH)
        {
            System.out.println("Pitch motor position below " + Constants.Turret.MIN_PITCH + " causing pitch motor to be disabled");
            pitchMotor.disable();
        }
        * /
    }


    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }
    
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }

    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }

    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public void setVoltage (Voltage voltage)
    {
        motor.setControl(new VoltageOut(voltage));
    }
    
    public void setDesiredPosition (Angle targetPosition)
    {
        double targetPositionRotations = targetPosition.in(Rotations);
        double currentPosition = motor.getPosition().getValue().in(Rotations);
        double rawDifference = targetPositionRotations - currentPosition;
        rawDifference %= 1;
        if (rawDifference > 0.5) rawDifference -= 1.0;
        else if (rawDifference < -0.5) rawDifference += 1.0;
        desiredPosition = currentPosition + rawDifference;
        motor.setControl(new MotionMagicVoltage(desiredPosition));
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
        simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Turret.GEAR_RATIO));
        simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Turret.GEAR_RATIO));

    }
    
    public boolean readyToShoot ()
    {
        return Math.abs(motor.getPosition().getValue().in(Rotation) - desiredPosition) < Constants.EPSILON;
    }

    public static Turret getInstance ()
    {
        if (instance == null) instance = new Turret();
        return instance;
    }

    */
}
