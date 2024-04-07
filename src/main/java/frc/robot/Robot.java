package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Gyro;
import frc.robot.swerve.SwerveConstraints;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwervePosition;

public class Robot extends TimedRobot {

  public SwerveManager swerveManager = new SwerveManager();
  public Field2d field2d = new Field2d();
  public Pose2d pose2d = new Pose2d();

  public StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

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
    field2d.setRobotPose(Gyro.ahrs.getDisplacementX(), Gyro.ahrs.getDisplacementY(), Gyro.getRotation2d());
    SmartDashboard.putData("Field", field2d);
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

    double xSpeed = -joy0.getRawAxis(1);
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

    publisher.set(moduleStates);
  }

  @Override
  public void disabledInit() {swerveManager.stopModules();}

  @Override
  public void disabledPeriodic() {}

  private SwerveModule testSwerve;

  @Override
  public void testInit() {
    joy0 = new Joystick(0);

    testSwerve = new SwerveModule(               "Swerve Teste",
            SwervePosition.FRONT_LEFT,
            10, // CAN ID drive Motor
            20, // CAN ID turning Motor
            false,
            false,
            1, // Analog Port of Absolute Encoder
            0,
            false);


  }

  @Override
  public void testPeriodic() {
    if(joy0.getRawButtonPressed(XboxController.Button.kB.value)) Gyro.zeroHeading();

    double speed = joy0.getRawAxis(1);
    double turn = joy0.getRawAxis(4);

    speed = Math.abs(speed) > SwerveConstraints.kDeadband ? speed : 0;
    turn = Math.abs(turn) > SwerveConstraints.kDeadband ? turn : 0;

    speed = xLimiter.calculate(speed) * SwerveConstraints.kTeleOpMaxAccel;
    turn = turnLimiter.calculate(turn) * SwerveConstraints.kTeleOpMaxRotRps;

    double rad = 0;
    rad += (testSwerve.getTurningPosition() + turn > 0 ? 5 : -5);
    if(turn == 0) rad = testSwerve.getTurningPosition();
    testSwerve.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromRadians(rad)));
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
