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
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;


public class ShooterSubsystem extends SubsystemBase {
    
    private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
    private final CANcoder hoodEncoder;
    private double VelocityVoltage = 0.0;
    private double PositionVoltage = 0.0;
    private double targetRollerSpeed, targetHoodAngle;

    public ShooterSubsystem() {
        CANBus canbus = Constants.Swerve.CAN_BUS;
         
        warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.Rollers.WARMUP_1_ID, canbus);
        warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.Rollers.WARMUP_2_ID, canbus);
        warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.Rollers.WARMUP_3_ID, canbus);

        shooter = warmup3; 

        hood = new LoggedTalonFX("HoodMotor", Constants.Shooter.Hood.HOOD_ID, canbus);

        Slot0Configs rollersSlot0Configs = new Slot0Configs()
        .withKV(Constants.Intake.Rollers.kV)
        .withKP(Constants.Intake.Rollers.kP)
        .withKI(Constants.Intake.Rollers.kI)
        .withKD(Constants.Intake.Rollers.kD);

        CurrentLimitsConfigs rollersClConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
        .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT);

        MotorOutputConfigs rollersOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
        
        TalonFXConfiguration rollersConfiguration =  new TalonFXConfiguration()
        .withSlot0(rollersSlot0Configs)
        .withCurrentLimits(rollersClConfigs)
        .withMotorOutput(rollersOutputConfigs);

        warmup1.getConfigurator().apply(rollersConfig);
        warmup2.getConfigurator().apply(rollersConfig);
        warmup3.getConfigurator().apply(rollersConfig);

        Follower follower = new Follower(Constants.Shooter.Rollers.WARMUP_3_ID, MotorAlignmentValue.Aligned);

        warmup1.setControl(follower);
        warmup2.setControl(follower);

        hoodEncoder = new CANcoder(Constants.Shooter.Hood.ENCODER_PORT, Constants.Swerve.CAN_BUS);

        Slot0Configs hoodSlot0Configs = new Slot0Configs()
        .withKV(Constants.Intake.Rollers.kV)
        .withKP(Constants.Intake.Rollers.kP)
        .withKI(Constants.Intake.Rollers.kI)
        .withKD(Constants.Intake.Rollers.kD);

        CurrentLimitsConfigs hoodClConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
        .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT); 

        MotorOutputConfigs hoodOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

        FeedbackConfigs hoodFeedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Shooter.Hood.ENCODER_ROTS_PER_HOOD_ROT)
            .withRotorToSensorRatio(Constants.Shooter.Hood.MOTOR_ROTS_PER_ENCODER_ROT);

        
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

  
        TalonFXConfiguration hoodConfiguration =  new TalonFXConfiguration()
        .withSlot0(hoodSlot0Configs)
        .withCurrentLimits(hoodClConfigs)
        .withMotorOutput(hoodOutputConfigs)
        .withFeedback(hoodFeedbackConfigs);


        hood.getConfigurator().apply(hoodConfiguration);

        MagnetSensorConfigs hoodCANcoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Shooter.Hood.ENCODER_OFFSET));

        
        hoodEncoder.getConfigurator().apply(hoodCANcoderConfig);


    }
}
