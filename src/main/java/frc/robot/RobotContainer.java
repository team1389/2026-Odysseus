package frc.robot;

// import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TestHood;
import frc.robot.commands.TestIntake;
import frc.robot.commands.TestIntakeArm;
import frc.robot.commands.TestShooter;
import frc.robot.commands.TestTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  // Drivetrain controls
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swer
  ve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverController = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Mechanisms controls
  // Define controller ports | DO NOT TOUCH |
  final CommandXboxController manipController = new CommandXboxController(1);

  public TurretSubsystem turretSubsystem;
  public FlywheelSubsystem flywheelSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public HoodSubsystem hoodSubsystem;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  // Creates Bindings for controllers
  public RobotContainer() {
    turretSubsystem = new TurretSubsystem();
    flywheelSubsystem = new FlywheelSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    hoodSubsystem = new HoodSubsystem();

    // Pathplanner Auto commands
    NamedCommands.registerCommand("moveIntake", intakeSubsystem.runRollers(70));
    NamedCommands.registerCommand("testShoot", Commands.print("Odysseus shoots a test shot."));
    NamedCommands.registerCommand("MoveIntakeArm", intakeSubsystem.setAngle(Degrees.of(RobotMap.IntakeArmAngle)));

    // NamedCommands.registerCommand("testShoot", Commands.runOnce(() -> {System.out.println("Robot
    // did a test shot.");}));

    autoChooser = AutoBuilder.buildAutoChooser("MoveFwd5mAuto");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {

    // double targetAngle = 45; // Set target angle for the turret

    // double armIntakeTargetAngle = 46;
    // double intakeTargetAngle = 90;
    // double outtakeTargetAngle = 0;

    if (DriverStation.isTest()) {
      // Testing subsytem commands
      // Turret
      // turretSubsystem.setDefaultCommand(
      //    new TestTurret(turretSubsystem, manipController.getLeftY()));
      manipController.povLeft().whileTrue(new TestTurret(turretSubsystem, 2));
      manipController.povRight().whileTrue(new TestTurret(turretSubsystem, -2));
      // Flywheel
      manipController.rightTrigger().whileTrue(new TestShooter(flywheelSubsystem, -60));
      manipController.rightBumper().whileTrue(new TestShooter(flywheelSubsystem, 60));
      // Intake
      manipController.a().whileTrue(new TestIntake(intakeSubsystem, 32));
      manipController.b().whileTrue(new TestIntake(intakeSubsystem, -32));
      // Hood
      manipController.povUp().whileTrue(new TestHood(hoodSubsystem, 1));
      manipController.povDown().whileTrue(new TestHood(hoodSubsystem, -1));
      // IntakeArm
      manipController.leftBumper().whileTrue(new TestIntakeArm(intakeSubsystem, 1));
      manipController.leftTrigger().whileTrue(new TestIntakeArm(intakeSubsystem, -1));

    } else {
      // Comp commands should be put here
      /*
      manipController.a().whileTrue(new RunIntake(intakeSubsystem, armIntakeTargetAngle));
      manipController.b().whileTrue(intakeSubsystem.intake(Degrees.of(intakeTargetAngle)));
      manipController.y().whileTrue(intakeSubsystem.outtake(Degrees.of(outtakeTargetAngle)));

      manipController.x().whileTrue(new RunTurret(turretSubsystem, targetAngle));

      manipController.rightBumper().whileTrue(new RunFlywheel(flywheelSubsystem));
       */
    }

    // Drivetrain commands
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    driverController
        .povUp()
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverController
        .povDown()
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController
        .back()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController
        .back()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController
        .start()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController
        .start()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
