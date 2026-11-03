package frc.robot;

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

public class Constants {
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
