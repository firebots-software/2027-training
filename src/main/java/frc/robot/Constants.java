package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class Swerve {
    public static final double FF_RADIUS_M2 = 0.1;
    public static final double MAX_TRANSLATIONAL_MOVEMENT_SQUARED = 0.0625;
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.COBRA;
    public static final CANBus CAN_BUS =
        new CANBus(WHICH_SWERVE_ROBOT.CANBUS_NAME, "./logs/example.hoot");
    // the distance over the bump in meters

    public static final double TARGET_POS_ERROR = 0.07;
    public static final double TARGET_ANGLE_ERROR = 0.1;
    public static final double MAX_HEADING_TRACKING_ROT_RATE_RADS_PER_SECOND = 4;

    public static enum SwerveLevel {
      L2(6.75, 21.428571428571427),
      L3(6.12, 21.428571428571427),
      FIVEN_L3(5.2734375, 26.09090909091);
      public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO;

      SwerveLevel(double drive, double steer) {
        DRIVE_GEAR_RATIO = drive;
        STEER_GEAR_RATIO = steer;
      }
    }

    public static enum SwerveDrivePIDValues {
      SERRANO(0.18014, 0d, 0d, -0.023265, 0.12681, 0.058864),
      PROTO(0.053218, 0d, 0d, 0.19977, 0.11198, 0.0048619),
      // JAMES_HARDEN(0.16901, 0d, 0d, 0.1593, 0.12143, 0.0091321); //0.041539
      // //0.12301
      JAMES_HARDEN(0.36, 0d, 0d, 0.2425, 0.11560693641, 0), // 0.041539 //0.12301
      COBRA(0.25, 0d, 0d, 0.2d, 0.124, 0d); // 0.041539 //0.12301
      public final double KP, KI, KD, KS, KV, KA;

      SwerveDrivePIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
      }
    }

    public static enum SwerveSteerPIDValues {
      SERRANO(50d, 0d, 0.2, 0d, 1.5, 0d),
      PROTO(20d, 0d, 0d, 0d, 0d, 0d),
      JAMES_HARDEN(38.982d, 2.4768d, 0d, 0.23791d, 0d, 0.1151d),
      COBRA(100d, 0d, 0.5, 0.1, 2.49, 0d);
      public final double KP, KI, KD, KS, KV, KA;

      SwerveSteerPIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
      }
    }

    public static enum SwerveDriveToPosePIDValues {
      SERRANO(3.067, 0, 0, 4.167, 0, 0, 3.667, 0, 0),
      PROTO(0, 0, 0, 0, 0, 0, 0, 0, 0),
      JAMES_HARDEN(0, 0, 0, 0, 0, 0, 0, 0, 0),
      COBRA(3.467, 0, 0, 3.567, 0, 0, 2.867, 0, 0);

      public final double kPX;
      public final double kIX;
      public final double kDX;
      public final double kPY;
      public final double kIY;
      public final double kDY;
      public final double kPR;
      public final double kIR;
      public final double kDR;

      SwerveDriveToPosePIDValues(
          double kPX,
          double kIX,
          double kDX,
          double kPY,
          double kIY,
          double kDY,
          double kPR,
          double kIR,
          double kDR) {
        this.kPX = kPX;
        this.kIX = kIX;
        this.kDX = kDX;
        this.kPY = kPY;
        this.kIY = kIY;
        this.kDY = kDY;
        this.kPR = kPR;
        this.kIR = kIR;
        this.kDR = kDR;
      }
    }

    public static enum SwerveDriveToPoseProfileValues {
      SERRANO(2, 2, 2, 2),
      PROTO(0.5, 0.5, 0.2, 0.2),
      JAMES_HARDEN(0.5, 0.5, 0.2, 0.2),
      COBRA(5, 8, 1.9, 10);
      public final double maxVelocityLinear,
          maxAccelerationLinear,
          maxVelocityAngular,
          maxAccelerationAngular;

      SwerveDriveToPoseProfileValues(
          double maxVelocityLinear,
          double maxAccelerationLinear,
          double maxVelocityAngular,
          double maxAccelerationAngular) {
        this.maxVelocityLinear = maxVelocityLinear;
        this.maxAccelerationLinear = maxAccelerationLinear;
        this.maxVelocityAngular = maxVelocityAngular;
        this.maxAccelerationAngular = maxAccelerationAngular;
      }
    }

    public static enum ChoreoPIDValues {
      SERRANO(0.1d, 0d, 0d, 0.1d, 0d, 0d, 3.867d, 0d, 0d),
      PROTO(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      JAMES_HARDEN(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      COBRA(3.467, 0, 0, 3.567, 0, 0, 2.5, 0, 0); // 3.75 m/s^2 max accel while turning max,
      public final double kPX, kIX, kDX, kPY, kIY, kDY, kPR, kIR, kDR;

      ChoreoPIDValues(
          double kPX,
          double kIX,
          double kDX,
          double kPY,
          double kIY,
          double kDY,
          double kPR,
          double kIR,
          double kDR) {
        this.kPX = kPX;
        this.kIX = kIX;
        this.kDX = kDX;
        this.kPY = kPY;
        this.kIY = kIY;
        this.kDY = kDY;
        this.kPR = kPR;
        this.kIR = kIR;
        this.kDR = kDR;
      }
    }

    public static enum RobotDimensions {
      SERRANO(Inches.of(22.52), Inches.of(22.834)), // length, width
      PROTO(Inches.of(22.52), Inches.of(22.834)), // length, width
      JAMES_HARDEN(Inches.of(26.75), Inches.of(22.75)), // length, width
      COBRA(Inches.of(29.0), Inches.of(26.0)); // length, width
      public final Distance length, width;

      RobotDimensions(Distance length, Distance width) {
        this.length = length;
        this.width = width;
      }
    }

    public static enum BumperThickness {
      SERRANO(Inches.of(2.625)), // thickness
      PROTO(Inches.of(2.625)), // thickness
      JAMES_HARDEN(Inches.of(3.313)), // thickness
      COBRA(Inches.of(0)); // thickness

      public final Distance thickness;

      BumperThickness(Distance thickness) {
        this.thickness = thickness;
      }
    }

    public static enum SwerveType {
      SERRANO(
          Rotations.of(-0.466552734375), // front left
          Rotations.of(-0.436767578125), // front right
          Rotations.of(-0.165283203125), // back left
          Rotations.of(-0.336181640625), // back right
          SwerveLevel.L3, // what level the swerve drive is
          SwerveDrivePIDValues.SERRANO,
          SwerveSteerPIDValues.SERRANO,
          SwerveDriveToPosePIDValues.SERRANO,
          SwerveDriveToPoseProfileValues.SERRANO,
          ChoreoPIDValues.SERRANO,
          RobotDimensions.SERRANO,
          "Patrice the Pineapple",
          BumperThickness.SERRANO,
          3.5714285714285716,
          true),
      PROTO(
          Rotations.of(0.3876953125), // front left
          Rotations.of(0.159912109375), // front right
          Rotations.of(0.213134765625), // back left
          Rotations.of(-0.3818359375), // back right
          SwerveLevel.L2, // what level the swerve drive is
          SwerveDrivePIDValues.PROTO,
          SwerveSteerPIDValues.PROTO,
          SwerveDriveToPosePIDValues.PROTO,
          SwerveDriveToPoseProfileValues.PROTO,
          ChoreoPIDValues.PROTO,
          RobotDimensions.PROTO,
          "rio",
          BumperThickness.PROTO,
          3.5714285714285716,
          true),
      JAMES_HARDEN(
          Rotations.of(-0.0834960938), // front left
          Rotations.of(-0.4912109375), // front right
          Rotations.of(0.1931152344), // back left
          Rotations.of(-0.15576171875), // back right
          SwerveLevel.L3,
          SwerveDrivePIDValues.JAMES_HARDEN,
          SwerveSteerPIDValues.JAMES_HARDEN,
          SwerveDriveToPosePIDValues.JAMES_HARDEN,
          SwerveDriveToPoseProfileValues.JAMES_HARDEN,
          ChoreoPIDValues.JAMES_HARDEN,
          RobotDimensions.JAMES_HARDEN,
          "JamesHarden",
          BumperThickness.JAMES_HARDEN,
          3.5714285714285716,
          true),
      COBRA(
          Rotations.of(0.096923828125), // front left, 21
          Rotations.of(0.03271484375), // front right, 22
          Rotations.of(0.02587890625), // back left, 20
          Rotations.of(-0.09765625), // back right, 23
          SwerveLevel.FIVEN_L3,
          SwerveDrivePIDValues.COBRA,
          SwerveSteerPIDValues.COBRA,
          SwerveDriveToPosePIDValues.COBRA,
          SwerveDriveToPoseProfileValues.COBRA,
          ChoreoPIDValues.COBRA,
          RobotDimensions.COBRA,
          "Viper",
          BumperThickness.COBRA,
          3.5714285714285716,
          false);
      public final Angle FRONT_LEFT_ENCODER_OFFSET,
          FRONT_RIGHT_ENCODER_OFFSET,
          BACK_LEFT_ENCODER_OFFSET,
          BACK_RIGHT_ENCODER_OFFSET;
      public final SwerveLevel SWERVE_LEVEL;
      public final SwerveDrivePIDValues SWERVE_DRIVE_PID_VALUES;
      public final SwerveSteerPIDValues SWERVE_STEER_PID_VALUES;
      public final SwerveDriveToPosePIDValues SWERVE_DRIVE_TO_POSE_PID_VALUES;
      public final SwerveDriveToPoseProfileValues SWERVE_DRIVE_TO_POSE_PROFILE_VALUES;
      public final ChoreoPIDValues CHOREO_PID_VALUES;
      public final RobotDimensions ROBOT_DIMENSIONS;
      public final String CANBUS_NAME;
      public final double COUPLE_RATIO;
      public final BumperThickness BUMPER_THICKNESS;
      public final boolean INVERTED_MODULES;

      SwerveType(
          Angle fl,
          Angle fr,
          Angle bl,
          Angle br,
          SwerveLevel swerveLevel,
          SwerveDrivePIDValues swerveDrivePIDValues,
          SwerveSteerPIDValues swerveSteerPIDValues,
          SwerveDriveToPosePIDValues swerveDriveToPosePIDValues,
          SwerveDriveToPoseProfileValues swerveDriveToPoseProfileValues,
          ChoreoPIDValues choreoPIDValues,
          RobotDimensions robotDimensions,
          String canbus_name,
          BumperThickness thickness,
          double coupled_ratio,
          boolean invertedModules) {
        FRONT_LEFT_ENCODER_OFFSET = fl;
        FRONT_RIGHT_ENCODER_OFFSET = fr;
        BACK_LEFT_ENCODER_OFFSET = bl;
        BACK_RIGHT_ENCODER_OFFSET = br;
        SWERVE_LEVEL = swerveLevel;
        SWERVE_DRIVE_PID_VALUES = swerveDrivePIDValues;
        SWERVE_STEER_PID_VALUES = swerveSteerPIDValues;
        SWERVE_DRIVE_TO_POSE_PID_VALUES = swerveDriveToPosePIDValues;
        SWERVE_DRIVE_TO_POSE_PROFILE_VALUES = swerveDriveToPoseProfileValues;
        CHOREO_PID_VALUES = choreoPIDValues;
        ROBOT_DIMENSIONS = robotDimensions;
        CANBUS_NAME = canbus_name;
        BUMPER_THICKNESS = thickness;
        COUPLE_RATIO = coupled_ratio;
        INVERTED_MODULES = invertedModules;
      }
    }

    // these outline the speed calculations
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.868;
    // 5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s =
    // 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND =
        26.971;

    public static class Auto {
      public static final double TIME_FOR_OUTPOST_INTAKE = 3.0;
      public static final double TIME_FOR_BUMP_FORWARDS = 0.95;
      public static final double TIME_FOR_BUMP_BACKWARDS = 1.05;
      public static final double TIME_FOR_BUMP_FORWARDS_SLOWER = 1.30;
      public static final double WAIT_TIME_FOR_ALLIANCE = 4.0;

      public static enum Intake {
        LeftIntakeSweepShort,
        RightIntakeSweepShort,
        RightSecondDip,
        LeftSecondDip, // outside in
        LeftSecondDipLong, // inside out
        RightSecondDipLong,
        HubLeft,
        HubRight
      }

      public static enum ShootPos {
        LeftShoot,
        RightShoot
      }

      public static enum ClimbPos {}

      public static enum Depot {
        DepotStart,
        DepotSweep
      }

      public static enum Outpost {
        OutpostStart,
        OutpostSweep,
        OutpostStartPush
      }

      public static enum MiscPaths {
        LeftShootToBump,
        RightShootToBump,
        Nike,
        anthony
      }
    }
  }
    private static final class RightCameraVals {
        public static final int X = 0;
        public static final int Y = 0;
        public static final int Z = 0;
        public static final int ROLL = 0;
        public static final int PITCH = 0;
        public static final int YAW = 0;
        private static final double FX = 0;
        private static final double CX = 0;
        private static final double FY = 0;
        private static final double CY = 0;
        private static final double K1 = 0;
        private static final double K2 = 0;
        private static final double P1 = 0;
        private static final double P2 = 0;
        private static final double K3 = 0;
        private static final double K4 = 0;
        private static final double K5 = 0;
        private static final double K6 = 0;
    }

    public static final class RightCamera {
    public static final Transform3d transform =
        new Transform3d(
            new Translation3d(RightCameraVals.X, RightCameraVals.Y, RightCameraVals.Z),
            new Rotation3d(RightCameraVals.ROLL, RightCameraVals.PITCH, RightCameraVals.YAW));
    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            RightCameraVals.FX,
            0.0,
            RightCameraVals.CX,
            0.0,
            RightCameraVals.FY,
            RightCameraVals.CY,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            RightCameraVals.K1,
            RightCameraVals.K2,
            RightCameraVals.P1,
            RightCameraVals.P2,
            RightCameraVals.K3,
            RightCameraVals.K4,
            RightCameraVals.K5,
            RightCameraVals.K6);
  }
}
