package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sensors.Gyro;
import frc.robot.swerve.SwerveConstraints;
import frc.robot.swerve.SwerveManager;

public class Robot extends TimedRobot {

  public SwerveManager swerveManager = new SwerveManager();

  @Override
  public void robotInit() {

    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
      Gyro.zeroHeading();
    }).start();

  }

  @Override
  public void robotPeriodic() {
    Gyro.periodic();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  Joystick joy0;
  SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstraints.kTeleOpMaxAccel);
  SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstraints.kTeleOpMaxAccel);
  SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstraints.kTeleOpMaxRot);

  @Override
  public void teleopInit() {

    joy0 = new Joystick(0);

  }

  @Override
  public void teleopPeriodic() {

    if(joy0.getRawButtonPressed(XboxController.Button.kB.value)) Gyro.zeroHeading();

    double xSpeed = joy0.getRawAxis(0);
    double ySpeed = -joy0.getRawAxis(0);
    double turningSpeed = joy0.getRawAxis(4);
    boolean fieldOrientedFn = !joy0.getRawButton(XboxController.Button.kA.value);

    xSpeed = Math.abs(xSpeed) > SwerveConstraints.kDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > SwerveConstraints.kDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > SwerveConstraints.kDeadband ? turningSpeed : 0;

    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstraints.kTeleOpMaxAccel;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstraints.kTeleOpMaxAccel;
    turningSpeed = turnLimiter.calculate(turningSpeed) * SwerveConstraints.kTeleOpMaxRotRps;

    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFn) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Gyro.getRotation2d());
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstraints.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveManager.setModuleStates(moduleStates);
  }

  @Override
  public void disabledInit() {swerveManager.stopModules();}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}