package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  private double targetDeg = Constants.Arm.DEFAULT_ARM_ANGLE;
  private LoggedTalonFX rt, rb, lt, lb;
  private LoggedTalonFX master;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armff;
  private MotionMagicConfigs mmc;

  private boolean initialized = false;
  private final double armHorizontalOffset =
      Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET / Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    
    // Starting gain from the Serrano reference; verify on the robot.
    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Arm.S0C_KP).withKI(0).withKD(0);
    
    armff = new ArmFeedforward(Constants.Arm.ARMFF_KS, Constants.Arm.ARMFF_KG, Constants.Arm.ARMFF_KV);

    rt = new LoggedTalonFX("ArmRightTop", Constants.Arm.RT_PORT, Constants.Arm.CANBUS_NAME);
    rb = new LoggedTalonFX("ArmRightBottom", Constants.Arm.RB_PORT, Constants.Arm.CANBUS_NAME);
    lt = new LoggedTalonFX("ArmLefttop", Constants.Arm.LT_PORT, Constants.Arm.CANBUS_NAME);
    lb = new LoggedTalonFX("ArmLeftBottom", Constants.Arm.LB_PORT, Constants.Arm.CANBUS_NAME);

    Follower follower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Aligned);
    Follower invertedFollower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Opposed);
    rt.setControl(invertedFollower);
    rb.setControl(invertedFollower);
    lb.setControl(follower);

    TalonFXConfigurator rtConfig = rt.getConfigurator();
    TalonFXConfigurator rbConfig = rb.getConfigurator();
    TalonFXConfigurator ltConfig = lt.getConfigurator();
    TalonFXConfigurator lbConfig = lb.getConfigurator();

    rtConfig.apply(moc); rbConfig.apply(moc); ltConfig.apply(moc); lbConfig.apply(moc);
    rtConfig.apply(clc); rbConfig.apply(clc); ltConfig.apply(clc); lbConfig.apply(clc);

    master = lt;
    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(s0c);

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    mmc.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    masterConfigurator.apply(mmc);

    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);

  }

  public static ArmSubsystem getInstance() {
    if (instance == null) { instance = new ArmSubsystem(); }
    return instance;
  }

  private double getAbsolutePosition() {
    return (revEncoder.get() - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL + Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET + 1d) % 1;
  }

  public void setAngle(double angle){
    setTargetDegrees(angle);
  }

  public void reset(){
    master.stopMotor();
    initialized = revEncoder.isConnected()
        && master.setPosition(getAbsolutePosition() * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR).isOK();
  }

  public boolean isInitialized() {
    return initialized;
  }

  public void setPosition(double angleDeg) {
    setTargetDegrees(angleDeg);
    if (!initialized) {
      master.stopMotor();
      return;
    }
    double currentRadians = (master.getPosition().getValueAsDouble()
        / Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR - armHorizontalOffset) * 2 * Math.PI;
    PositionVoltage m_PositionVoltage = new PositionVoltage(calcIntegrateTarRots(targetDeg))
        .withFeedForward(armff.calculate(currentRadians, 0));
    master.setControl(m_PositionVoltage);
  }

  public void resetPosition() {
    reset();
  }

  private double calcIntegrateTarRots(double angleDeg) {
    double armRots = angleDeg / 360d + armHorizontalOffset;
    return armRots * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
  }

  public void setTargetDegrees(double deg) {
    this.targetDeg = deg;
  }

  public boolean atTarget(double tol) {
    var position = master.getPosition();
    return initialized && position.getStatus().isOK() && Math.abs(
        targetDeg - (position.getValueAsDouble() / Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR * 360d
            - armHorizontalOffset * 360d)) < tol;
  }

  @Override
  public void periodic() {
    if (!initialized) {
      reset();
    }
    setPosition(targetDeg);
  }
}
