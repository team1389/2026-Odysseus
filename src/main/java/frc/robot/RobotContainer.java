package frc.robot;

// import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootOnMoveCmd;
import frc.robot.commands.TestHood;
import frc.robot.commands.TestIntake;
import frc.robot.commands.TestIntakeArm;
import frc.robot.commands.TestSerializer;
import frc.robot.commands.TestTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class RobotContainer {

  // Drivetrain controls
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private double slowModeScale = 0.45; // scaling factor of drive speed in slow mode

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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Mechanisms controls
  // Define controller ports | DO NOT TOUCH |
  private final CommandXboxController driverController = new CommandXboxController(0);
  final CommandXboxController manipController = new CommandXboxController(1);
  public TurretSubsystem turretSubsystem;
  public FlywheelSubsystem flywheelSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public HoodSubsystem hoodSubsystem;
  public VisionSubsystem visionSubsystem;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;
  public SerializerSubsystem serializerSubsystem;

  // Creates Bindings for controllers
  public RobotContainer() {
    turretSubsystem = new TurretSubsystem();
    flywheelSubsystem = new FlywheelSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    hoodSubsystem = new HoodSubsystem();
    visionSubsystem = new VisionSubsystem();
    serializerSubsystem = new SerializerSubsystem();

    // Pathplanner Auto commands
    NamedCommands.registerCommand("testShoot", Commands.print("Odysseus shoots a test shot."));
    NamedCommands.registerCommand(
        "moveIntake", new TestIntake(intakeSubsystem, 3).withTimeout(2)); // Runs for 2 seconds);
    NamedCommands.registerCommand(
        "MoveIntakeArm", new TestIntakeArm(intakeSubsystem, () -> -2.0).withTimeout(2.0));
    NamedCommands.registerCommand(
        "shootOnTheMove",
        new ShootOnMoveCmd(
                turretSubsystem,
                flywheelSubsystem,
                hoodSubsystem,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                () -> AllianceFlipUtil.flip(FieldConstants.blueHub))
            .alongWith(
                new WaitCommand(3)
                    .andThen(new TestSerializer(serializerSubsystem, -32))
                    .withTimeout(10)));

    autoChooser = AutoBuilder.buildAutoChooser("Comp-MovingBackFromCenter");
    SmartDashboard.putData("Auto Mode", autoChooser);

    SmartDashboard.putNumber("targetSpeed", 0);
    SmartDashboard.putNumber("setHoodAngle", 0);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  public void configureBindings() {
    // Turret
    manipController.povLeft().whileTrue(new TestTurret(turretSubsystem, -5));
    manipController.povRight().whileTrue(new TestTurret(turretSubsystem, 5));

    // Flywheel
    manipController
        .a()
        .whileTrue(flywheelSubsystem.setVelocity(() -> RPM.of(1900)))
        .whileFalse(flywheelSubsystem.setDutyCycle(0));

    manipController
        .b()
        .whileTrue(flywheelSubsystem.setVelocity(() -> RPM.of(-1900)))
        .whileFalse(flywheelSubsystem.setDutyCycle(0));

    // Intake
    manipController.leftBumper().whileTrue(new TestIntake(intakeSubsystem, 12));
    manipController.leftTrigger().whileTrue(new TestIntake(intakeSubsystem, -12));
    // Hood
    manipController.povUp().whileTrue(new TestHood(hoodSubsystem, () -> Degrees.of(45)));
    manipController.povDown().whileTrue(new TestHood(hoodSubsystem, () -> Degrees.of(22)));

    // Shoot on the move
    manipController
        .x()
        .whileTrue(
            new ShootOnMoveCmd(
                turretSubsystem,
                flywheelSubsystem,
                hoodSubsystem,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                () -> AllianceFlipUtil.flip(FieldConstants.blueHub)));
    // IntakeArm
    intakeSubsystem.setDefaultCommand(
        new TestIntakeArm(intakeSubsystem, () -> -manipController.getLeftY() * 0.625));

    // Serializer
    manipController.rightBumper().whileTrue(new TestSerializer(serializerSubsystem, -32));
    manipController.rightTrigger().whileTrue(new TestSerializer(serializerSubsystem, 32));
    // Drivetrain commands
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -(driverController.rightBumper().getAsBoolean() // slow mode
                                ? scaleAndSmooth(driverController.getLeftY(), slowModeScale)
                                // scaling and square smoothing in slow mode
                                : driverController.getLeftY())
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -(driverController.rightBumper().getAsBoolean() // slow mode
                                ? scaleAndSmooth(driverController.getLeftX(), slowModeScale)
                                // scaling and square smoothing in slow mode
                                : driverController.getLeftX())
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

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureTestBindings() {
    // Testing subsytem commands

    // Turret
    manipController.povLeft().whileTrue(new TestTurret(turretSubsystem, -5));
    manipController.povRight().whileTrue(new TestTurret(turretSubsystem, 5));

    // Flywheel
    manipController
        .a()
        .whileTrue(flywheelSubsystem.setVelocity(() -> RPM.of(1900)))
        .whileFalse(flywheelSubsystem.setDutyCycle(0));

    // Intake
    manipController.leftBumper().whileTrue(new TestIntake(intakeSubsystem, 10));
    manipController.leftTrigger().whileTrue(new TestIntake(intakeSubsystem, -10));
    // Hood
    manipController.povUp().whileTrue(new TestHood(hoodSubsystem, () -> Degrees.of(45)));
    manipController.povDown().whileTrue(new TestHood(hoodSubsystem, () -> Degrees.of(22)));

    // Shoot on the move
    manipController
        .x()
        .whileTrue(
            new ShootOnMoveCmd(
                turretSubsystem,
                flywheelSubsystem,
                hoodSubsystem,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                () -> AllianceFlipUtil.flip(FieldConstants.blueHub)));
    // IntakeArm
    intakeSubsystem.setDefaultCommand(
        new TestIntakeArm(intakeSubsystem, () -> -manipController.getLeftY()));

    // Serializer
    manipController.rightBumper().whileTrue(new TestSerializer(serializerSubsystem, -32));
    manipController.rightTrigger().whileTrue(new TestSerializer(serializerSubsystem, 32));

    // Drivetrain commands
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -(driverController.rightBumper().getAsBoolean() // slow mode
                                ? scaleAndSmooth(driverController.getLeftY(), slowModeScale)
                                // scaling and square smoothing in slow mode
                                : driverController.getLeftY())
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -(driverController.rightBumper().getAsBoolean() // slow mode
                                ? scaleAndSmooth(driverController.getLeftX(), slowModeScale)
                                // scaling and square smoothing in slow mode
                                : driverController.getLeftX())
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

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public Pose2d getHubPose() {
    return new Pose2d(Inches.of(469.11), Inches.of(158.84), Rotation2d.kZero);
  }

  private double scaleAndSmooth(double inputValue, double scaleFactor) {
    return inputValue * Math.abs(inputValue) * scaleFactor;
  }
}
