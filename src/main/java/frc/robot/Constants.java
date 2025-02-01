package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
//  Sets the slew rate limit
    public static final int SLEWRATELIMITER = 3;
//  Sets P and D values of the PIDs 
    public static final double DRIVE_P_VALUE = 0.062203; 
    public static final double DRIVE_D_VALUE = 0; 
    public static final double STEER_P_VALUE = 22.942; 
    public static final double STEER_D_VALUE = 0.85373; 
//  Sets the KS value
    public static final double STEER_S_VALUE = 0.13962;

/// Vision Constants
    public static class VisionConstants {
        public static final String kCameraName = "dumbdumbcamera";
        

        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.22, 0.0, 0.22 ),
            new Rotation3d(0,Units.degreesToRadians(-15),0));
     
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public  static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public  static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}

