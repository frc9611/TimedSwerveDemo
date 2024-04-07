 package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstraints {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/5.8462;
    public static final double kTurningMotorGearRatio = 1/18;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * Math.PI * 2;
    public static final double kDriveEncoderRPM2Mps = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2Rps = kTurningEncoderRot2Rad / 60;

    public static final double kPTurning = .5;
    public static final double kMaxSpeedMps = 3;
    public static final double kMaxSpeedRps = 20;

    public static final double kDeadband = .01;
    public static final double kTeleOpMaxAccel = .6;
    public static final double kTeleOpMaxAccelMps = kTeleOpMaxAccel * kMaxSpeedMps;
    public static final double kTeleOpMaxRot = .6;
    public static final double kTeleOpMaxRotRps = kTeleOpMaxRot * kMaxSpeedRps;

    public static final double kRobotTrackWidth = Units.inchesToMeters(21); // distance between left & right wheels
    public static final double kRobotWheelBase = Units.inchesToMeters(25.5); // distance between front & back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kRobotWheelBase / 2, -kRobotTrackWidth / 2),
        new Translation2d(kRobotWheelBase / 2, kRobotTrackWidth / 2),
        new Translation2d(-kRobotWheelBase / 2, -kRobotTrackWidth / 2),
        new Translation2d(-kRobotWheelBase / 2, kRobotTrackWidth / 2)
    );
}
