package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants.TunerConstants;



public class Modules {

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(TunerConstants.kCANBus.getName())
        .withPigeon2Id(TunerConstants.kPigeonId)
        .withPigeon2Configs(TunerConstants.pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(TunerConstants.kDriveGearRatio)
            .withSteerMotorGearRatio(TunerConstants.kSteerGearRatio)
            .withCouplingGearRatio(TunerConstants.kCoupleRatio)
            .withWheelRadius(TunerConstants.kWheelRadius)
            .withSteerMotorGains(TunerConstants.steerGains)
            .withDriveMotorGains(TunerConstants.driveGains)
            .withSteerMotorClosedLoopOutput(TunerConstants.kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(TunerConstants.kDriveClosedLoopOutput)
            .withSlipCurrent(TunerConstants.kSlipCurrent)
            .withSpeedAt12Volts(TunerConstants.kSpeedAt12Volts)
            .withDriveMotorType(TunerConstants.kDriveMotorType)
            .withSteerMotorType(TunerConstants.kSteerMotorType)
            .withFeedbackSource(TunerConstants.kSteerFeedbackType)
            .withDriveMotorInitialConfigs(TunerConstants.driveInitialConfigs)
            .withSteerMotorInitialConfigs(TunerConstants.steerInitialConfigs)
            .withEncoderInitialConfigs(TunerConstants.encoderInitialConfigs)
            .withSteerInertia(TunerConstants.kSteerInertia)
            .withDriveInertia(TunerConstants.kDriveInertia)
            .withSteerFrictionVoltage(TunerConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(TunerConstants.kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            TunerConstants.kFrontLeftSteerMotorId, TunerConstants.kFrontLeftDriveMotorId, TunerConstants.kFrontLeftEncoderId, TunerConstants.kFrontLeftEncoderOffset,
            TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kFrontLeftSteerMotorInverted,TunerConstants.kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            TunerConstants.kFrontRightSteerMotorId,TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightEncoderId, TunerConstants.kFrontRightEncoderOffset,
            TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kFrontRightSteerMotorInverted, TunerConstants.kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            TunerConstants.kBackLeftSteerMotorId, TunerConstants.kBackLeftDriveMotorId, TunerConstants.kBackLeftEncoderId, TunerConstants.kBackLeftEncoderOffset,
            TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos, TunerConstants.kInvertLeftSide, TunerConstants.kBackLeftSteerMotorInverted, TunerConstants.kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            TunerConstants.kBackRightSteerMotorId, TunerConstants.kBackRightDriveMotorId, TunerConstants.kBackRightEncoderId, TunerConstants.kBackRightEncoderOffset,
            TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos, TunerConstants.kInvertRightSide, TunerConstants.kBackRightSteerMotorInverted, TunerConstants.kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static Drivetrain createDrivetrain() {
        return new Drivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
