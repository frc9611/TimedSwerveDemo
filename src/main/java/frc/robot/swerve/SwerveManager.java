package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Gyro;

import java.util.ArrayList;
import java.util.Collection;

public class SwerveManager {
    public ArrayList<SwerveModule> swerves = new ArrayList<>();

    public SwerveManager() {
        SwerveModule frontLeft = new SwerveModule(
               "Swerve Frontal Esquerdo",
                SwervePosition.FRONT_LEFT,
                10, // CAN ID drive Motor
                20, // CAN ID turning Motor
                true,
                false,
                1, // Analog Port of Absolute Encoder
                0,
                true
        );

        SwerveModule frontRight = new SwerveModule(
                "Swerve Frontal Direito",
                SwervePosition.FRONT_RIGHT,
                11, // CAN ID drive Motor
                21, // CAN ID turning Motor
                false,
                false,
                2, // Analog Port of Absolute Encoder
                0,
                false
        );

        SwerveModule backLeft = new SwerveModule(
                "Swerve Traseiro Esquerdo",
                SwervePosition.BACK_LEFT,
                12, // CAN ID drive Motor
                22, // CAN ID turning Motor
                true,
                false,
                3, // Analog Port of Absolute Encoder
                0,
                true
        );

        SwerveModule backRight = new SwerveModule(
                "Swerve Traseiro Direito",
                SwervePosition.BACK_RIGHT,
                13, // CAN ID drive Motor
                23, // CAN ID turning Motor
                false,
                false,
                4, // Analog Port of Absolute Encoder
                0,
                false
        );

        swerves.add(frontLeft);
        swerves.add(frontRight);
        swerves.add(backLeft);
        swerves.add(backRight);

    }

    public void stopModules() {
        for(SwerveModule swerve : swerves) {
            swerve.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstraints.kMaxSpeedMps);
        for(int i = 0; i < swerves.size(); i++) {
            swerves.get(i).setDesiredState(desiredStates[i]);
        }
    }

}
