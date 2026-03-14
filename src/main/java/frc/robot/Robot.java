package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/** Don't change the name of this class since the VM is set up to run this */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * Initialize all systems here as public & static. Ex: public static System system = new System();
   */
  @Override
  public void robotInit() {}

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    for (Optional<EstimatedRobotPose> poseEstimate :
        m_robotContainer.visionSubsystem.getPoseEstimates()) {
      if (!poseEstimate.isEmpty()) {
        Pose2d newPose2d =
            new Pose2d(
                poseEstimate.get().estimatedPose.getTranslation().toTranslation2d(),
                m_robotContainer.drivetrain.getRotation3d().toRotation2d());
        m_robotContainer.drivetrain.addVisionMeasurement(
            poseEstimate.get().estimatedPose.toPose2d(), poseEstimate.get().timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    m_robotContainer.configureBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    m_robotContainer.configureTestBindings();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Hood Angle", m_robotContainer.hoodSubsystem.getAngleDegrees());
    SmartDashboard.putNumber("Turret Angle", m_robotContainer.turretSubsystem.getAngleDegrees());
    SmartDashboard.putNumber("Flywheel Speed", m_robotContainer.flywheelSubsystem.getSpeedRPM());
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
