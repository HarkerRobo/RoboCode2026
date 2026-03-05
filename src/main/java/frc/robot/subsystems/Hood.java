package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;
import frc.robot.simulation.StallSimulator;
import frc.robot.util.Util;

/**
 * 
 */
public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX motor;

    private double desiredPosition; // rotations
    
    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.001, Constants.Hood.GEAR_RATIO),
        DCMotor.getKrakenX44(1));

        /*
    private final SingleJointedArmSim motorSimModel = new SingleJointedArmSim(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX44Foc(1), 0.001, Constants.Hood.GEAR_RATIO),
            DCMotor.getKrakenX44Foc(1)).getGearbox(),
        Constants.Hood.GEAR_RATIO,
        Constants.Hood.MOMENT_OF_INERTIA,
        Constants.Hood.LENGTH,
        Units.degreesToRadians(Constants.Hood.MIN_ANGLE),
        Units.degreesToRadians(Constants.Hood.MAX_ANGLE),
        false,
        Units.degreesToRadians(10.0)
        // no std devs -> no noise simulated
        );
        */



    private Hood()
    {
        motor = new TalonFX(Constants.Hood.ID, Constants.CAN_SUPERSTRUCTURE);

        config();

        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();
            simState.Orientation = Constants.Hood.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
        }
    }

    private void config()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (!isSimulated())
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
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        
        config.Slot0.kS = Constants.Hood.KS;
        config.Slot0.kV = Constants.Hood.KV;
        config.Slot0.kA = Constants.Hood.KA;
        config.Slot0.kG = Constants.Hood.KG;

        //config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);
    }

    public void moveToPosition(Angle desiredPosition)
    {
        this.desiredPosition = desiredPosition.in(Rotations);
        motor.setControl(new PositionVoltage(desiredPosition));
    }

    public void moveToEffectivePosition(Angle desiredPosition)
    {
        moveToPosition(effectiveToMechanism(desiredPosition));
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

    public void setPosition(Angle position)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        motor.setPosition(position, 0.5);
    }

    public void setVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        motor.setControl(new VelocityVoltage(velocity));
    }

    public void setDutyCycle(double dutyCycle)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        motor.setControl(new DutyCycleOut(dutyCycle));
    }

    public void setVoltage(Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hood");
            return;
        }
        motor.setVoltage(voltage.in(Volts));
    }

    
    public boolean readyToShoot ()
    {
        return Math.abs(motor.getPosition().getValue().in(Rotations) - desiredPosition) < Degrees.of(0.25).in(Rotations);
    }
    
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }
    
    public boolean isStalling()
    {
        return Math.abs(motor.getStatorCurrent().getValueAsDouble()) >= Constants.Hood.STALLING_CURRENT;
    }

    public double getStatorCurrent()
    {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public long lastTime = System.currentTimeMillis();

    @Override
    public void periodic()
    {
        if (isSimulated())
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
            simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.Hood.GEAR_RATIO));
            simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.Hood.GEAR_RATIO));
        }
    }

    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOOD_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOOD_INDEX) == SubsystemStatus.Disabled;
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.1).div(Seconds.of(1.)),Volts.of(0.7),Seconds.of(5.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Hood")
                .voltage(getVoltage())
                .angularPosition(getPosition())
                .angularVelocity(getVelocity()),
        this)
    );

    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId D" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    

    public static Hood getInstance()
    {
        if (instance == null) instance = new Hood();
        return instance;
    }

    public static Angle effectiveToMechanism(Angle effectivePitch)
    {
        return Degrees.of(75.0).minus(effectivePitch);
    }

    public static Angle mechanismToEffective(Angle mechanismPitch)
    {
        return Degrees.of(75.0).minus(mechanismPitch);
    }

    public static double effectiveToMechanism(double effectivePitchDegrees)
    {
        return 75.0 - effectivePitchDegrees;
    }

    public static double mechanismToEffective(double mechanismPitchDegrees)
    {
        return 75.0 - mechanismPitchDegrees;
    }
}
