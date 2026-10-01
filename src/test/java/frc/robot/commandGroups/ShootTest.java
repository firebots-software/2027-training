package frc.robot.commandGroups;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootTest {
  private static final double SHOT_ANGLE = 35;
  private static final double FIRST_SPEED = 40;
  private static final double SECOND_SPEED = 70;
  private static FakePeter peter;
  private static FakeArm arm;
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  @BeforeAll
  static void createSubsystems() {
    assertTrue(HAL.initialize(500, 0));
    peter = new FakePeter();
    arm = new FakeArm();
  }

  @BeforeEach
  void enableScheduler() {
    scheduler.cancelAll();
    scheduler.enable();
    peter.resetState();
    arm.resetState();
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void disableScheduler() {
    scheduler.cancelAll();
    SimHooks.resumeTiming();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void closeSubsystems() throws Exception {
    CommandScheduler.getInstance().unregisterSubsystem(peter, arm);
    closeHardware(peter, PeterSubsystem.class);
    closeHardware(arm, ArmSubsystem.class);
  }

  @Test
  void doesNotFeedBeforeBothMechanismsAreReady() {
    Shoot shoot = startShot();
    runFor(0.2);
    assertTrue(scheduler.isScheduled(shoot));
    assertEquals(0, peter.feedRuns);
    assertTrue(peter.shooterRunning);
    assertEquals(SHOT_ANGLE, arm.target);

    peter.wheelsAtSpeed = true;
    runFor(0.2);
    assertEquals(0, peter.feedRuns, "Ready wheels must still wait for the arm");
  }

  @Test
  void keepsShootingAngleWhenArmArrivesBeforeShooter() {
    Shoot shoot = startShot();
    arm.position = SHOT_ANGLE;
    runFor(0.2);

    assertTrue(scheduler.isScheduled(shoot));
    assertEquals(SHOT_ANGLE, arm.target, "Finishing the arm command must hold its shooting angle");
    assertEquals(0, peter.feedRuns);

    peter.wheelsAtSpeed = true;
    runFor(0.2);
    assertTrue(peter.preshooterRunning);
    assertEquals(SHOT_ANGLE, arm.target);
  }

  @Test
  void skipsFeedIfWheelsLoseSpeedWhileWaitingForArm() {
    Shoot shoot = startShot();
    peter.wheelsAtSpeed = true;
    runFor(0.2);
    assertTrue(peter.shooterRunning);
    assertEquals(0, peter.feedRuns);

    peter.wheelsAtSpeed = false;
    arm.position = SHOT_ANGLE;
    runFor(0.3);

    assertEquals(0, peter.feedRuns, "Recheck wheel speed at the feed handoff");
    assertFalse(peter.shooterRunning);
    assertEquals(Constants.Arm.DEFAULT_ARM_ANGLE, arm.target);
    arm.position = Constants.Arm.DEFAULT_ARM_ANGLE;
    runFor(0.2);
    assertFalse(scheduler.isScheduled(shoot));
  }

  @Test
  void feedsOneDetectedNoteThenStopsAndReturnsToNeutral() {
    Shoot shoot = startShot();
    arm.position = SHOT_ANGLE;
    peter.wheelsAtSpeed = true;
    peter.notePresent = true;
    runFor(0.2);

    assertTrue(peter.preshooterRunning);
    assertTrue(peter.shooterRunning);
    assertEquals(FIRST_SPEED, peter.firstSpeed);
    assertEquals(SECOND_SPEED, peter.secondSpeed);
    assertEquals(SHOT_ANGLE, arm.target);

    peter.notePresent = false;
    runFor(0.2);
    assertFalse(peter.preshooterRunning);
    assertFalse(peter.shooterRunning);
    assertFalse(peter.intakeRunning);
    assertEquals(Constants.Arm.DEFAULT_ARM_ANGLE, arm.target);

    arm.position = Constants.Arm.DEFAULT_ARM_ANGLE;
    runFor(0.2);
    assertFalse(scheduler.isScheduled(shoot));
  }

  @Test
  void cancellationStopsLatchedShooterAfterWarmupAlreadyFinished() {
    Shoot shoot = startShot();
    peter.wheelsAtSpeed = true;
    runFor(0.2);
    assertTrue(peter.shooterRunning);
    int warmupRuns = peter.shooterRuns;
    runFor(0.2);
    assertEquals(warmupRuns, peter.shooterRuns,
        "The wheel command should have finished while the arm is still moving");

    scheduler.cancel(shoot);

    assertFalse(scheduler.isScheduled(shoot));
    assertFalse(peter.shooterRunning);
    assertFalse(peter.preshooterRunning);
    assertFalse(peter.intakeRunning);
    assertEquals(Constants.Arm.DEFAULT_ARM_ANGLE, arm.target);
  }

  @Test
  void schedulingAgainWaitsForReadinessAndFeedsANewNote() {
    Shoot shoot = startShot();
    arm.position = SHOT_ANGLE;
    peter.wheelsAtSpeed = true;
    peter.notePresent = true;
    runFor(0.2);
    assertTrue(peter.preshooterRunning);
    peter.notePresent = false;
    runFor(0.2);
    arm.position = Constants.Arm.DEFAULT_ARM_ANGLE;
    runFor(0.2);
    assertFalse(scheduler.isScheduled(shoot));
    int firstShotFeedRuns = peter.feedRuns;

    peter.notePresent = true;
    scheduler.schedule(shoot);
    runFor(0.2);

    assertEquals(firstShotFeedRuns, peter.feedRuns,
        "A completed earlier shot must not bypass preparation of the next shot");
    assertTrue(scheduler.isScheduled(shoot));
    assertFalse(peter.preshooterRunning);

    arm.position = SHOT_ANGLE;
    runFor(0.2);
    assertTrue(peter.preshooterRunning);
    peter.notePresent = false;
    runFor(0.2);
    assertFalse(peter.preshooterRunning);
    assertFalse(peter.shooterRunning);
    arm.position = Constants.Arm.DEFAULT_ARM_ANGLE;
    runFor(0.2);
    assertFalse(scheduler.isScheduled(shoot));
  }

  @Test
  void failedArmResetWaitsWithoutSpinningOrFeedingUntilCancelled() {
    arm.resetSucceeds = false;
    Shoot shoot = startShot();
    runFor(0.2);

    assertTrue(scheduler.isScheduled(shoot));
    assertEquals(0, peter.shooterRuns);
    assertEquals(0, peter.feedRuns);
    scheduler.cancel(shoot);
    assertFalse(scheduler.isScheduled(shoot));
    assertFalse(peter.shooterRunning);
    assertFalse(peter.preshooterRunning);
    assertEquals(Constants.Arm.DEFAULT_ARM_ANGLE, arm.target);
  }

  @Test
  void missingBeamTransitionTimesOutFeedThenWaitsForNeutral() {
    Shoot shoot = startShot();
    arm.position = SHOT_ANGLE;
    peter.wheelsAtSpeed = true;
    peter.notePresent = false;
    runFor(0.6);

    assertTrue(peter.preshooterRunning,
        "An initially clear beam is not evidence that the note has exited");
    assertTrue(scheduler.isScheduled(shoot));

    runFor(2.0);
    assertFalse(peter.preshooterRunning);
    assertFalse(peter.shooterRunning);
    assertEquals(Constants.Arm.DEFAULT_ARM_ANGLE, arm.target);

    assertTrue(scheduler.isScheduled(shoot));
    arm.position = Constants.Arm.DEFAULT_ARM_ANGLE;
    runFor(0.2);
    assertFalse(scheduler.isScheduled(shoot));
  }

  private Shoot startShot() {
    Shoot shoot = new Shoot(peter, arm, SHOT_ANGLE, FIRST_SPEED, SECOND_SPEED);
    scheduler.schedule(shoot);
    return shoot;
  }

  private void runFor(double seconds) {
    int ticks = (int) Math.ceil(seconds / 0.02);
    for (int i = 0; i < ticks; i++) {
      SimHooks.stepTiming(0.02);
      scheduler.run();
    }
  }

  private static void closeHardware(Object subsystem, Class<?> type) throws Exception {
    Set<Object> closed = Collections.newSetFromMap(new IdentityHashMap<>());
    for (Field field : type.getDeclaredFields()) {
      if (Modifier.isStatic(field.getModifiers())) {
        continue;
      }
      field.setAccessible(true);
      Object value = field.get(subsystem);
      if (value instanceof AutoCloseable resource && closed.add(value)) {
        resource.close();
      }
    }
  }

  private static class FakeArm extends ArmSubsystem {
    double target;
    double position;
    boolean initialized;
    boolean resetSucceeds;

    void resetState() {
      target = Constants.Arm.DEFAULT_ARM_ANGLE;
      position = Constants.Arm.DEFAULT_ARM_ANGLE;
      initialized = false;
      resetSucceeds = true;
    }

    @Override
    public void reset() {
      initialized = resetSucceeds;
    }

    @Override
    public boolean isInitialized() {
      return initialized;
    }

    @Override
    public void setTargetDegrees(double angle) {
      target = angle;
    }

    @Override
    public boolean atTarget(double tolerance) {
      return initialized && Math.abs(target - position) < tolerance;
    }

    @Override
    public void periodic() {}
  }

  private static class FakePeter extends PeterSubsystem {
    boolean wheelsAtSpeed;
    boolean shooterRunning;
    boolean preshooterRunning;
    boolean intakeRunning;
    boolean notePresent;
    double firstSpeed;
    double secondSpeed;
    int shooterRuns;
    int feedRuns;

    void resetState() {
      wheelsAtSpeed = false;
      shooterRunning = false;
      preshooterRunning = false;
      intakeRunning = false;
      notePresent = false;
      firstSpeed = 0;
      secondSpeed = 0;
      shooterRuns = 0;
      feedRuns = 0;
    }

    @Override
    public void runShooter(double first, double second) {
      firstSpeed = first;
      secondSpeed = second;
      shooterRunning = true;
      shooterRuns++;
    }

    @Override
    public void runShooter() {
      runShooter(Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS,
          Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS);
    }

    @Override
    public boolean isShooterReady() {
      return shooterRunning && wheelsAtSpeed;
    }

    @Override
    public void stopShooter() {
      shooterRunning = false;
    }

    @Override
    public void spinUpPreShooterVelocity() {
      preshooterRunning = true;
      feedRuns++;
    }

    @Override
    public void runPreShooter(double speed) {
      preshooterRunning = speed != 0;
      feedRuns++;
    }

    @Override
    public void stopPreShooterMotor() {
      preshooterRunning = false;
    }

    @Override
    public void stopPreshooter() {
      stopPreShooterMotor();
    }

    @Override
    public void stopIntake() {
      intakeRunning = false;
    }

    @Override
    public boolean notePresent() {
      return notePresent;
    }

    @Override
    public boolean isNotePresent() {
      return notePresent();
    }

    @Override
    public void periodic() {}
  }
}
