package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.FeedNote;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PeterCommandLifecycleTest {
  private static CountingPeter peter;
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  @BeforeAll
  static void initializeHardware() {
    assertTrue(HAL.initialize(500, 0));
    peter = new CountingPeter();
  }

  @AfterAll
  static void closeHardware() throws Exception {
    CommandScheduler.getInstance().unregisterSubsystem(peter);
    for (Field field : PeterSubsystem.class.getDeclaredFields()) {
      field.setAccessible(true);
      if (field.get(peter) instanceof AutoCloseable resource) {
        resource.close();
      }
    }
  }

  @BeforeEach
  void enableScheduler() {
    scheduler.cancelAll();
    scheduler.enable();
    peter.resetCounts();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
  }

  @AfterEach
  void disableScheduler() {
    scheduler.cancelAll();
    SimHooks.resumeTiming();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void warmupDoesNotFeedAndKeepsSpeedForHandoff() {
    SpinUpShooter warmup = new SpinUpShooter(peter, 40, 45);
    scheduler.schedule(warmup);

    assertEquals(1, peter.intakeStops);
    assertEquals(1, peter.preshooterStops);
    scheduler.run();
    scheduler.run();
    assertTrue(scheduler.isScheduled(warmup));
    assertEquals(2, peter.shooterRuns);
    assertEquals(40, peter.shooter1RPS);
    assertEquals(45, peter.shooter2RPS);
    assertEquals(0, peter.preshooterRuns);
    assertEquals(0, peter.shooterStops);

    peter.ready = true;
    scheduler.run();

    assertFalse(scheduler.isScheduled(warmup));
    assertEquals(0, peter.shooterStops);
    assertEquals(0, peter.preshooterRuns);
  }

  @Test
  void conflictingPeterCommandInterruptsWarmupAndStopsShooter() {
    SpinUpShooter warmup = new SpinUpShooter(peter);
    scheduler.schedule(warmup);
    scheduler.run();
    assertEquals(Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS, peter.shooter1RPS);
    assertEquals(Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS, peter.shooter2RPS);

    Command otherPeterCommand = new RunCommand(() -> {}, peter);
    scheduler.schedule(otherPeterCommand);

    assertFalse(scheduler.isScheduled(warmup));
    assertTrue(scheduler.isScheduled(otherPeterCommand));
    assertEquals(1, peter.shooterStops);
    assertEquals(0, peter.preshooterRuns);
  }

  @Test
  void backupWaitsForSuccessfulPositionResetBeforeMovingOrFinishing() {
    peter.positionResetSucceeds = false;
    peter.backedUp = true;
    BackupPeter backup = new BackupPeter(peter);
    scheduler.schedule(backup);
    scheduler.run();

    assertTrue(scheduler.isScheduled(backup));
    assertEquals(0, peter.reverseRuns);
    assertEquals(1, peter.preshooterStops);

    peter.positionResetSucceeds = true;
    peter.backedUp = false;
    scheduler.run();
    scheduler.run();
    assertTrue(scheduler.isScheduled(backup));
    assertEquals(1, peter.reverseRuns);

    peter.backedUp = true;
    scheduler.run();
    assertFalse(scheduler.isScheduled(backup));
    assertEquals(2, peter.preshooterStops);
  }

  @Test
  void feedingMaintainsRequestedSpeedsAndWaitsForBlockedThenClearBeam() {
    FeedNote feed = new FeedNote(peter, 40, 45);
    scheduler.schedule(feed);
    scheduler.run();

    assertTrue(scheduler.isScheduled(feed));
    assertEquals(40, peter.shooter1RPS);
    assertEquals(45, peter.shooter2RPS);
    assertEquals(1, peter.preshooterRuns);

    peter.notePresent = true;
    scheduler.run();
    assertTrue(scheduler.isScheduled(feed));

    peter.notePresent = false;
    scheduler.run();
    assertFalse(scheduler.isScheduled(feed));
    assertEquals(1, peter.shooterStops);
    assertEquals(2, peter.intakeStops);
    assertEquals(2, peter.preshooterStops);
  }

  @Test
  void feedingTimesOutWithNoNoteOrStuckBeam() {
    for (boolean blocked : new boolean[] {false, true}) {
      peter.resetCounts();
      peter.notePresent = blocked;
      FeedNote feed = new FeedNote(peter, 40, 45);
      scheduler.schedule(feed);
      scheduler.run();
      SimHooks.stepTiming(1.9);
      scheduler.run();
      assertTrue(scheduler.isScheduled(feed));

      SimHooks.stepTiming(0.11);
      scheduler.run();
      assertFalse(scheduler.isScheduled(feed));
      assertEquals(1, peter.shooterStops);
      assertEquals(2, peter.intakeStops);
      assertEquals(2, peter.preshooterStops);
    }
  }

  @Test
  void feedingCancellationStopsMotorsAndReuseResetsSensorAndTimerState() {
    peter.notePresent = true;
    FeedNote feed = new FeedNote(peter, 40, 45);
    scheduler.schedule(feed);
    scheduler.run();
    SimHooks.stepTiming(1.5);
    scheduler.run();
    scheduler.cancel(feed);
    assertEquals(1, peter.shooterStops);
    assertEquals(2, peter.intakeStops);
    assertEquals(2, peter.preshooterStops);

    peter.notePresent = false;
    scheduler.schedule(feed);
    scheduler.run();
    SimHooks.stepTiming(1);
    scheduler.run();
    assertTrue(scheduler.isScheduled(feed));

    peter.notePresent = true;
    scheduler.run();
    peter.notePresent = false;
    scheduler.run();
    assertFalse(scheduler.isScheduled(feed));
    assertEquals(2, peter.shooterStops);
  }

  private static class CountingPeter extends PeterSubsystem {
    int intakeStops;
    int preshooterStops;
    int preshooterRuns;
    int shooterRuns;
    int shooterStops;
    int reverseRuns;
    boolean ready;
    boolean positionResetSucceeds;
    boolean backedUp;
    boolean notePresent;
    double shooter1RPS;
    double shooter2RPS;

    void resetCounts() {
      intakeStops = 0;
      preshooterStops = 0;
      preshooterRuns = 0;
      shooterRuns = 0;
      shooterStops = 0;
      reverseRuns = 0;
      ready = false;
      positionResetSucceeds = true;
      backedUp = false;
      notePresent = false;
      shooter1RPS = 0;
      shooter2RPS = 0;
    }

    @Override
    public void stopIntake() {
      intakeStops++;
    }

    @Override
    public void stopPreShooterMotor() {
      preshooterStops++;
    }

    @Override
    public void spinUpPreShooterVelocity() {
      preshooterRuns++;
    }

    @Override
    public void runShooter(double shooter1RPS, double shooter2RPS) {
      shooterRuns++;
      this.shooter1RPS = shooter1RPS;
      this.shooter2RPS = shooter2RPS;
    }

    @Override
    public void stopShooter() {
      shooterStops++;
    }

    @Override
    public boolean isShooterReady() {
      return ready;
    }

    @Override
    public boolean notePresent() {
      return notePresent;
    }

    @Override
    public boolean resetPreshooterPos() {
      return positionResetSucceeds;
    }

    @Override
    public void reversePreshooter(double count) {
      reverseRuns++;
    }

    @Override
    public boolean isBackedUp(double count) {
      return backedUp;
    }

    @Override
    public void stopPreshooter() {
      preshooterStops++;
    }
  }
}
