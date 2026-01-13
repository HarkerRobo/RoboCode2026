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
// todo - code cancoder
public class Turret extends SubsystemBase
{
    private static Turret instance;

    private TalonFX yawMotor;

    private TalonFX pitchMotor; // for the hood

    private DCMotorSim yawSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Turret.YAW_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
    
    private DCMotorSim pitchSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Turret.PITCH_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
    
    private SysIdRoutine yawRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(((Voltage v) -> driveYaw(v)), null, this));

    public String yawSysIdCommand = "none";
    private Function<String, Runnable> yawSysIdCommandSetterFactory = (String s) -> (()->{yawSysIdCommand=s;});
    
    public Command yawQuasistaticForward = yawRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        .beforeStarting(yawSysIdCommandSetterFactory.apply("quasistatic-forward")).finallyDo(yawSysIdCommandSetterFactory.apply("none")).withName("SysId QF");
    public Command yawQuasistaticReverse = yawRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        .beforeStarting(yawSysIdCommandSetterFactory.apply("quasistatic-reverse")).finallyDo(yawSysIdCommandSetterFactory.apply("none")).withName("SysId QR");
    public Command yawDynamicForward = yawRoutine.dynamic(SysIdRoutine.Direction.kForward)
        .beforeStarting(yawSysIdCommandSetterFactory.apply("dynamic-forward")).finallyDo(yawSysIdCommandSetterFactory.apply("none")).withName("SysId DF");
    public Command yawDynamicReverse = yawRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        .beforeStarting(yawSysIdCommandSetterFactory.apply("dynamic-reverse")).finallyDo(yawSysIdCommandSetterFactory.apply("none")).withName("SysId DR");

    
    
    private SysIdRoutine pitchRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(((Voltage v) -> drivePitch(v)), null, this));
    
    private Function<String, Runnable> pitchSysIdCommandSetterFactory = (String s) -> (()->{pitchSysIdCommand=s;});
    
    public String pitchSysIdCommand = "none";
    
    public Command pitchQuasistaticForward = pitchRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        .beforeStarting(pitchSysIdCommandSetterFactory.apply("quasistatic-forward")).finallyDo(pitchSysIdCommandSetterFactory.apply("none")).withName("SysId QF");
    public Command pitchQuasistaticReverse = pitchRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        .beforeStarting(pitchSysIdCommandSetterFactory.apply("quasistatic-reverse")).finallyDo(pitchSysIdCommandSetterFactory.apply("none")).withName("SysIr QR");
    public Command pitchDynamicForward = pitchRoutine.dynamic(SysIdRoutine.Direction.kForward)
        .beforeStarting(pitchSysIdCommandSetterFactory.apply("dynamic-forward")).finallyDo(pitchSysIdCommandSetterFactory.apply("none")).withName("SysId DF");
    public Command pitchDynamicReverse = pitchRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        .beforeStarting(pitchSysIdCommandSetterFactory.apply("dynamic-reverse")).finallyDo(pitchSysIdCommandSetterFactory.apply("none")).withName("SysId DR");

    private Turret ()
    {
        yawMotor = new TalonFX(Constants.Turret.YAW_MOTOR_ID);
        pitchMotor = new TalonFX(Constants.Turret.PITCH_MOTOR_ID);


        config();

        if (Robot.isSimulation())
        {
            yawMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            yawMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            pitchMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            pitchMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }

        yawMotor.setPosition(Telemetry.getInstance().turretYawRawSubscriber.get());
        System.out.println("Read yaw position: " + Telemetry.getInstance().turretYawRawSubscriber.get());
    }

    private void config ()
    {
        TalonFXConfiguration yawConfig = new TalonFXConfiguration();
        yawConfig.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT;
        yawConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        yawConfig.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT;
        yawConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        yawConfig.Feedback.SensorToMechanismRatio = Constants.Turret.YAW_GEAR_RATIO;

        yawConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.MM_YAW_CRUISE_VELOCITY;
        yawConfig.MotionMagic.MotionMagicAcceleration = Constants.Turret.MM_YAW_ACCELERATION;
        yawConfig.MotionMagic.MotionMagicJerk = Constants.Turret.MM_YAW_JERK;

        yawConfig.MotorOutput.Inverted = Constants.Turret.YAW_INVERTED;
        yawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        yawConfig.Slot0.kP = Constants.Turret.YAW_KP;
        yawConfig.Slot0.kI = Constants.Turret.YAW_KI;
        yawConfig.Slot0.kD = Constants.Turret.YAW_KD;

        yawConfig.Slot0.kV = Constants.Turret.YAW_KV;
        yawConfig.Slot0.kG = Constants.Turret.YAW_KG;

        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.YAW_FORWARD_SOFTWARE_LIMIT_THRESHOLD;
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.YAW_REVERSE_SOFTWARE_LIMIT_THRESHOLD;
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        yawConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        yawConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        yawMotor.getConfigurator().apply(yawConfig);

        TalonFXConfiguration pitchConfig = new TalonFXConfiguration();
        pitchConfig.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT;
        pitchConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pitchConfig.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT;
        pitchConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        pitchConfig.Feedback.SensorToMechanismRatio = Constants.Turret.PITCH_GEAR_RATIO;
        
        pitchConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.MM_PITCH_CRUISE_VELOCITY;
        pitchConfig.MotionMagic.MotionMagicAcceleration = Constants.Turret.MM_PITCH_ACCELERATION;
        pitchConfig.MotionMagic.MotionMagicJerk = Constants.Turret.MM_PITCH_JERK;

        pitchConfig.MotorOutput.Inverted = Constants.Turret.PITCH_INVERTED;
        pitchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pitchConfig.Slot0.kP = Constants.Turret.PITCH_KP;
        pitchConfig.Slot0.kI = Constants.Turret.PITCH_KI;
        pitchConfig.Slot0.kD = Constants.Turret.PITCH_KD;

        pitchConfig.Slot0.kV = Constants.Turret.PITCH_KV;
        pitchConfig.Slot0.kG = Constants.Turret.PITCH_KG;

        pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.PITCH_FORWARD_SOFTWARE_LIMIT_THRESHOLD;
        pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        
        pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.PITCH_REVERSE_SOFTWARE_LIMIT_THRESHOLD;
        pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pitchConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        pitchConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        pitchMotor.getConfigurator().apply(pitchConfig);
    }

    @Override
    public void periodic ()
    {
    }


    /**
     * @return degrees
     */
    public double getYaw ()
    {
        return yawMotor.getPosition().getValue().in(Degrees);
    }
    
    /**
     * @return degrees
     */
    public double getPitch ()
    {
        return pitchMotor.getPosition().getValue().in(Degrees);
    }

    /**
     * For saving the value of the yaw position in nonvolatile memory
     */
    public double getRawYaw ()
    {
        return yawMotor.getPosition().getValueAsDouble();
    }

    public void driveYaw (Voltage voltage)
    {
        yawMotor.setControl(new VoltageOut(voltage));
    }
    
    public void drivePitch (Voltage voltage)
    {
        pitchMotor.setControl(new VoltageOut(voltage));
    }
    
    public void driveYaw (double dutyCycle)
    {
        yawMotor.setControl(new DutyCycleOut(dutyCycle));
    }
    
    public void drivePitch (double dutyCycle)
    {
        pitchMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    /**
     * @param targetYaw degrees
     */
    public void setTargetYaw (double targetYaw)
    {
        double currentPosition = yawMotor.getPosition().getValue().in(Rotations);
        double targetYawRotations = Units.degreesToRotations(targetYaw);
        double rawDifference = targetYawRotations - currentPosition;
        rawDifference %= 1;
        if (rawDifference > 0.5) rawDifference -= 1.0;
        else if (rawDifference < -0.5) rawDifference += 1.0;
        System.out.printf("%f - %f = %f -> %f\n", targetYawRotations, yawMotor.getPosition().getValue().in(Rotations), targetYawRotations - yawMotor.getPosition().getValue().in(Rotations), rawDifference);
        yawMotor.setControl(new MotionMagicVoltage(currentPosition + rawDifference));
    }
    
    /**
     * @param targetPitch radians
     */
    public void setTargetPitch (double targetPitch)
    {
        if (targetPitch < 0.0 || targetPitch > Constants.Turret.MAX_PITCH)
        {
            throw new RuntimeException("Improper targetPitch " + targetPitch + ".");
        }

        double targetPitchRotations = Units.degreesToRotations(targetPitch);
        pitchMotor.setControl(new MotionMagicVoltage(targetPitchRotations));
    }

    public void maintainPitch ()
    {
        setTargetPitch(getPitch());
    }

    @Override
    public void simulationPeriodic ()
    {
        TalonFXSimState yawSimState = yawMotor.getSimState();

        // set the supply voltage of the TalonFX
        yawSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        yawSim.setInputVoltage(yawSimState.getMotorVoltageMeasure().in(Volts));
        yawSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        yawSimState.setRawRotorPosition(yawSim.getAngularPosition().times(Constants.Turret.YAW_GEAR_RATIO));
        yawSimState.setRotorVelocity(yawSim.getAngularVelocity().times(Constants.Turret.YAW_GEAR_RATIO));


        TalonFXSimState pitchSimState = pitchMotor.getSimState();

        // set the supply voltage of the TalonFX
        pitchSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        pitchSim.setInputVoltage(pitchSimState.getMotorVoltageMeasure().in(Volts));
        pitchSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        pitchSimState.setRawRotorPosition(pitchSim.getAngularPosition().times(Constants.Turret.PITCH_GEAR_RATIO));
        pitchSimState.setRotorVelocity(pitchSim.getAngularVelocity().times(Constants.Turret.PITCH_GEAR_RATIO));
    }

    public static Turret getInstance ()
    {
        if (instance == null) instance = new Turret();
        return instance;
    }
}
