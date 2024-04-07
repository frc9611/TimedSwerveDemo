package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    public static void zeroHeading() {
        ahrs.reset();
    }

    public static double getHeading() {
        return Math.IEEEremainder(ahrs.getAngle(), 360);
    }

    public static Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public static void periodic() {
        SmartDashboard.putNumber("Robot Heading", Gyro.getHeading());
    }
}
