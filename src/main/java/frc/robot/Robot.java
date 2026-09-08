package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/** Don't change the name of this class since the VM is set up to run this */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    configureLogging();
  }
  
  private void configureLogging(){
    if(isReal()){
      //log to the roborio and publish live data
      //setting up advantage kit logging
    if(RobotBase.isReal()){
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    }
    else{
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,"_sim")));
    }
    }
    else{
      //replay exisiting log for simulation
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "sim")));

    }
    Logger.start();
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
    CommandScheduler.getInstance().run();
    Logger.recordOutput("Robot/Enabled", isEnabled());
    for (Optional<EstimatedRobotPose> poseEstimate :
        m_robotContainer.visionSubsystem.getPoseEstimates()) {
      if (!poseEstimate.isEmpty()) {
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
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Hood Angle", m_robotContainer.hoodSubsystem.getAngleDegrees());
    SmartDashboard.putNumber("Turret Angle", m_robotContainer.turretSubsystem.getAngleDegrees());
    SmartDashboard.putNumber("Flywheel Speed", m_robotContainer.flywheelSubsystem.getSpeedRPM());
  }

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
