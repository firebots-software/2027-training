package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;


public class ShooterSubsystem extends SubsystemBase {
    
    private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
    private final CANcoder hoodEncoder;

    VelocityVoltage mVelocityVoltage = new VelocityVoltage(0.0);
    PositionVoltage mPositionVoltage = new PositionVoltage(0.0);

    double targetRoller;
    double targetAngle;

    public ShooterSubsystem() {
        CANBus canbus = Constants.Swerve.CAN_BUS;
         
        warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.Rollers.WARMUP_1_ID, canbus);
        warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.Rollers.WARMUP_2_ID, canbus);
        warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.Rollers.WARMUP_3_ID, canbus);

        shooter = warmup3;

        hood = new LoggedTalonFX("Hood", Constants.Shooter.Hood.HOOD_ID, canbus);

        Slot0Configs rollersSlot0Configs = new Slot0Configs();
        rollersSlot0Configs.kP = Constants.Shooter.Rollers.KP;
        rollersSlot0Configs.kI = Constants.Shooter.Rollers.KI;
        rollersSlot0Configs.kD = Constants.Shooter.Rollers.KD;
        rollersSlot0Configs.kV = Constants.Shooter.Rollers.KV;
        rollersSlot0Configs.kS = Constants.Shooter.Rollers.KS;

        CurrentLimitsConfigs rollersCLConfigs = new CurrentLimitsConfigs();
        rollersCLConfigs.StatorCurrentLimit = Constants.Shooter.Rollers.STATOR_CURRENT_LIMIT;
        rollersCLConfigs.SupplyCurrentLimit = Constants.Shooter.Rollers.SUPPLY_CURRENT_LIMIT;

        MotorOutputConfigs rollersOutputConfigs = new MotorOutputConfigs();
        rollersOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rollersOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
        rollersConfig.Slot0 = rollersSlot0Configs;
        rollersConfig.CurrentLimits = rollersCLConfigs;
        rollersConfig.MotorOutput = rollersOutputConfigs;

        warmup1.getConfigurator().apply(rollersConfig);
        warmup2.getConfigurator().apply(rollersConfig);
        warmup3.getConfigurator().apply(rollersConfig);

        Follower follower = new Follower(Constants.Shooter.Rollers.WARMUP_3_ID, MotorAlignmentValue.Aligned);
        warmup1.setControl(follower);
        warmup2.setControl(follower);

        hoodEncoder = new CANcoder(Constants.Shooter.Hood.ENCODER_PORT, Constants.Swerve.CAN_BUS);

        Slot0Configs hoodSlot0Configs = new Slot0Configs();
        hoodSlot0Configs.kP = Constants.Shooter.Hood.KP;
        hoodSlot0Configs.kI = Constants.Shooter.Hood.KI;
        hoodSlot0Configs.kD = Constants.Shooter.Hood.KD;
        hoodSlot0Configs.kV = Constants.Shooter.Hood.KV;
        hoodSlot0Configs.kS = Constants.Shooter.Hood.KS;
        hoodSlot0Configs.kG = Constants.Shooter.Hood.KG;

        CurrentLimitsConfigs hoodClConfigs = new CurrentLimitsConfigs();
        hoodClConfigs.StatorCurrentLimit = Constants.Shooter.Hood.STATOR_CURRENT_LIMIT;
        hoodClConfigs.SupplyCurrentLimit = Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT;

        MotorOutputConfigs hoodOutputConfigs = new MotorOutputConfigs();
        hoodOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        FeedbackConfigs hoodFeedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Shooter.Hood.ENCODER_ROTS_PER_HOOD_ROT)
            .withRotorToSensorRatio(Constants.Shooter.Hood.MOTOR_ROTS_PER_ENCODER_ROT);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0 = hoodSlot0Configs;
        hoodConfig.CurrentLimits = hoodClConfigs;
        hoodConfig.MotorOutput = hoodOutputConfigs;

        hood.getConfigurator().apply(hoodConfig);

        MagnetSensorConfigs hoodCANcoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Shooter.Hood.ENCODER_OFFSET));
                
        hoodEncoder.getConfigurator().apply(hoodCANcoderConfig);

    }
}
