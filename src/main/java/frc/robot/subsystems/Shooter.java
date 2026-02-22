
package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

/**
 * This class controls the powering of the flywheels to propel the fuel
 */
public class Shooter extends SubsystemBase
{
    private static Shooter instance;

    private TalonFX leftMaster;
    private TalonFX leftFollower;
    
    private TalonFX rightMaster;
    private TalonFX rightFollower;
    
    private DCMotorSim leftSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(2));
    
    private DCMotorSim rightSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.001, Constants.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(2));

    private double targetVelocity = 0.0;

    private Shooter()
    {
        leftMaster = new TalonFX(Constants.Shooter.LEFT_MASTER_ID);
        leftFollower = new TalonFX(Constants.Shooter.LEFT_FOLLOWER_ID);
        rightMaster = new TalonFX(Constants.Shooter.RIGHT_MASTER_ID);
        rightFollower = new TalonFX(Constants.Shooter.RIGHT_FOLLOWER_ID);

        config();
        
        if (isSimulated())
        {
            leftMaster.getSimState().Orientation = Constants.Shooter.MECHANICAL_ORIENTATION;
            leftMaster.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        
    }

    private void config()
    {
        leftMaster.clearStickyFaults();
        leftFollower.clearStickyFaults();
        rightMaster.clearStickyFaults();
        rightFollower.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        if (isSimulated())
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


        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        leftMaster.getConfigurator().apply(config);
        leftFollower.getConfigurator().apply(config);
        rightMaster.getConfigurator().apply(config);
        rightFollower.getConfigurator().apply(config);

        Slot0Configs leftConfig = new Slot0Configs();
        leftConfig.kP = Constants.Shooter.LEFT_KP;
        leftConfig.kI = Constants.Shooter.LEFT_KP;
        leftConfig.kD = Constants.Shooter.LEFT_KP;
        leftConfig.kS = Constants.Shooter.LEFT_KS;
        leftConfig.kV = Constants.Shooter.LEFT_KV;
        leftConfig.kA = Constants.Shooter.LEFT_KA;
        leftMaster.getConfigurator().apply(leftConfig);
        
        Slot0Configs rightConfig = new Slot0Configs();
        rightConfig.kP = Constants.Shooter.RIGHT_KP;
        rightConfig.kI = Constants.Shooter.RIGHT_KP;
        rightConfig.kD = Constants.Shooter.RIGHT_KP;
        rightConfig.kS = Constants.Shooter.RIGHT_KS;
        rightConfig.kV = Constants.Shooter.RIGHT_KV;
        rightConfig.kA = Constants.Shooter.RIGHT_KA;
        rightMaster.getConfigurator().apply(rightConfig);

        leftFollower.setControl(new Follower(Constants.Shooter.LEFT_MASTER_ID, MotorAlignmentValue.Aligned));
        rightFollower.setControl(new Follower(Constants.Shooter.RIGHT_MASTER_ID, MotorAlignmentValue.Aligned));
    }
    
    public Voltage getLeftVoltage()
    {
        return leftMaster.getMotorVoltage().getValue();
    }
    
    public Voltage getRightVoltage()
    {
        return rightMaster.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getLeftVelocity()
    {
        return leftMaster.getVelocity().getValue();
    }
    
    public AngularVelocity getRightVelocity()
    {
        return rightMaster.getVelocity().getValue();
    }

    public void setVelocity (AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        leftMaster.setControl(new VelocityVoltage(velocity));
        rightMaster.setControl(new VelocityVoltage(velocity));
        targetVelocity = velocity.in(Rotations.per(Second));
    }

    public void setLeftVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        leftMaster.setControl(new VelocityVoltage(velocity));
    }

    public void setRightVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        rightMaster.setControl(new VelocityVoltage(velocity));
    }

    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        leftMaster.setControl(new VoltageOut(voltage));
        rightMaster.setControl(new VoltageOut(voltage));
    }

    public void setDutyCycle (double dutyCycle)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Shooter");
            return;
        }
        leftMaster.setControl(new DutyCycleOut(dutyCycle));
        rightMaster.setControl(new DutyCycleOut(dutyCycle));
    }
    
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState leftSimState = leftMaster.getSimState();

            // set the supply voltage of the TalonFX
            leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            leftSim.setInputVoltage(leftSimState.getMotorVoltageMeasure().in(Volts));
            leftSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            leftSimState.setRawRotorPosition(leftSim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
            leftSimState.setRotorVelocity(leftSim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));

            TalonFXSimState rightSimState = rightMaster.getSimState();

            // set the supply voltage of the TalonFX
            rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            rightSim.setInputVoltage(rightSimState.getMotorVoltageMeasure().in(Volts));
            rightSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            rightSimState.setRawRotorPosition(rightSim.getAngularPosition().times(Constants.Shooter.GEAR_RATIO));
            rightSimState.setRotorVelocity(rightSim.getAngularVelocity().times(Constants.Shooter.GEAR_RATIO));
        }
    }
    
    private SysIdRoutine leftSysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->leftMaster.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
                .voltage(getLeftVoltage())
                .angularPosition(leftMaster.getPosition().getValue())
                .angularVelocity(getLeftVelocity()),
        this)
    );
    
    public boolean readyToShoot()
    {
        return Math.abs(leftMaster.getVelocity().getValue().in(Rotations.per(Second)) - targetVelocity) < Constants.EPSILON &&
               Math.abs(rightMaster.getVelocity().getValue().in(Rotations.per(Second)) - targetVelocity) < Constants.EPSILON;
    }
    
    public boolean readyToShoot(double targetVelocity)
    {
        return Math.abs(leftMaster.getVelocity().getValue().in(Rotations.per(Second)) - targetVelocity) < Constants.EPSILON &&
               Math.abs(rightMaster.getVelocity().getValue().in(Rotations.per(Second)) - targetVelocity) < Constants.EPSILON;
    }

    public Command leftSysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return leftSysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    public Command leftSysIdDynamic (SysIdRoutine.Direction direction)
    {
        return leftSysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }

    
    private SysIdRoutine rightSysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->rightMaster.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Shooter")
                .voltage(getRightVoltage())
                .angularPosition(rightMaster.getPosition().getValue())
                .angularVelocity(getRightVelocity()),
        this)
    );

    public Command rightSysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return rightSysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    public Command rightSysIdDynamic (SysIdRoutine.Direction direction)
    {
        return rightSysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.SHOOTER_INDEX) == SubsystemStatus.Disabled;
    }
    
    public static Shooter getInstance()
    {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}                 
