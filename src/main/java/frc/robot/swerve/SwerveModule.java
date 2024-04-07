package frc.robot.swerve;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private final String name;
    private final SwervePosition position;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final SparkRelativeEncoder driveEncoder;
    private final SparkRelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final AnalogInput analogAbsoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    public SwerveModule(String name, SwervePosition position, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.name = name;
        this.position = position;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        analogAbsoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = (SparkRelativeEncoder) driveMotor.getEncoder();
        turningEncoder = (SparkRelativeEncoder) driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveConstraints.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveConstraints.kDriveEncoderRPM2Mps);

        turningEncoder.setVelocityConversionFactor(SwerveConstraints.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(SwerveConstraints.kTurningEncoderRPM2Rps);

        turningPidController = new PIDController(SwerveConstraints.kPTurning,0,0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // says to WPILib that our wheel is circular.

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = analogAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1 : 1);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / SwerveConstraints.kMaxSpeedMps);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString(name + "-state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
