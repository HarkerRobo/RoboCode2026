package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

/**
 * 
 */
public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX master;
    private static TalonFX follower;

    private double desiredPosition; // rotations
    
    private final SingleJointedArmSim motorSimModel = new SingleJointedArmSim(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.001, Constants.Hood.GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2)).getGearbox(),
        Constants.Hood.GEAR_RATIO,
        Constants.Hood.MOMENT_OF_INERTIA,
        Constants.Hood.LENGTH,
        Units.degreesToRadians(Constants.Hood.MIN_ANGLE),
        Units.degreesToRadians(Constants.Hood.MAX_ANGLE),
        true,
        Units.degreesToRadians(10.0)
        // no std devs -> no noise simulated
        );



    private Hood()
    {
        master = new TalonFX((RobotContainer.disableHood ? 100 : 0) + Constants.Hood.MASTER_ID);
        follower = new TalonFX((RobotContainer.disableHood ? 100 : 0) + Constants.Hood.FOLLOWER_ID);

        config();

        if (RobotContainer.simulateHood)
        {
            TalonFXSimState simState = master.getSimState();
            simState.Orientation = Constants.Hood.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        master.clearStickyFaults();
        follower.clearStickyFaults();
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (!RobotContainer.simulateHood)
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

        master.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

    }

    public void moveToPosition(Angle desiredPosition)
    {
        //System.out.println("Moving to position: " + desiredPosition.in(Degrees) + "°");
        this.desiredPosition = desiredPosition.in(Rotations);
        master.setControl(new MotionMagicVoltage(desiredPosition));
    }
    
    public Angle getPosition()
    {
        return master.getPosition().getValue();
    }
    
    public Voltage getVoltage()
    {
        return master.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return master.getVelocity().getValue();
    }

    public void setPosition(Angle position)
    {
        master.setPosition(position);
    }

    public void setVelocity(AngularVelocity velocity)
    {
        master.setControl(new VelocityVoltage(velocity));
    }

    public void setDutyCycle(double dutyCycle)
    {
        master.setControl(new DutyCycleOut(dutyCycle));
    }

    public void setVoltage(Voltage voltage)
    {
        master.setVoltage(voltage.in(Volts));
    }

    
    public boolean readyToShoot ()
    {
        return Math.abs(master.getPosition().getValue().in(Rotations) - desiredPosition) < Constants.EPSILON;
    }
    
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }
    
    public boolean isStalling()
    {
        return Math.abs(master.getStatorCurrent().getValueAsDouble()) >= Constants.Hood.STALLING_CURRENT;
    }


    @Override
    public void periodic()
    {
        if (RobotContainer.simulateHood)
        {
            TalonFXSimState simState = master.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage motorVoltage = simState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            motorSimModel.setInputVoltage(motorVoltage.in(Volts));
            motorSimModel.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(
                    Units.radiansToRotations(motorSimModel.getAngleRads()) * Constants.Hood.GEAR_RATIO);
            simState.setRotorVelocity(
                    Units.radiansToRotations(motorSimModel.getVelocityRadPerSec()) * Constants.Hood.GEAR_RATIO);
        }
    }

    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.05).div(Seconds.of(1.)),Volts.of(0.1),Seconds.of(10.0)), 
        new SysIdRoutine.Mechanism((Voltage v)->master.setControl(new VoltageOut(v)),
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
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    

    public static Hood getInstance()
    {
        if (instance == null) instance = new Hood();
        return instance;
    }
}
