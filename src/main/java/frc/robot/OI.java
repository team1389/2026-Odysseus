package frc.robot;

// import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.command.RunFlywheel;
// import frc.command.RunIntake;
// import frc.command.RunTurret;
import frc.command.TestHood;
import frc.command.TestIntake;
import frc.command.TestIntakeArm;
import frc.command.TestSerializer;
import frc.command.TestShooter;
import frc.command.TestTurret;
import frc.command.joystickHoodCommand;
import frc.command.joystickTurretCommand;
import frc.subsystems.FlywheelSubsystem;
import frc.subsystems.HoodSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.SerializerSubsystem;
import frc.subsystems.TurretSubsystem;

public class OI {
  // Define controller ports | DO NOT TOUCH |
  final CommandXboxController manipController = new CommandXboxController(1);
  final CommandXboxController driveController = new CommandXboxController(0);
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  public static IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  public static HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public static SerializerSubsystem serializerSubsystem = new SerializerSubsystem();

  // Creates Bindings for controllers
  public OI() {
    configureBindings();
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
      manipController.povLeft().whileTrue(new TestTurret(turretSubsystem, 5));
      manipController.povRight().whileTrue(new TestTurret(turretSubsystem, -5));
      turretSubsystem.setDefaultCommand(
          new joystickTurretCommand(turretSubsystem, () -> -manipController.getRightX() * 10));

      // Flywheel
      manipController.rightTrigger().whileTrue(new TestShooter(flywheelSubsystem, -16));
      manipController.rightBumper().whileTrue(new TestShooter(flywheelSubsystem, 16));
      // Intake
      manipController.a().whileTrue(new TestIntake(IntakeSubsystem, 32));
      manipController.b().whileTrue(new TestIntake(IntakeSubsystem, -32));
      // Hood
      manipController.povUp().whileTrue(new TestHood(hoodSubsystem, 1));
      manipController.povDown().whileTrue(new TestHood(hoodSubsystem, -1));
      hoodSubsystem.setDefaultCommand(
          new joystickHoodCommand(hoodSubsystem, () -> -manipController.getRightY() * 10));
      // IntakeArm
      manipController.leftBumper().whileTrue(new TestIntakeArm(IntakeSubsystem, 16));
      manipController.leftTrigger().whileTrue(new TestIntakeArm(IntakeSubsystem, -16));
      // Serializer
      manipController.start().whileTrue(new TestSerializer(serializerSubsystem, 5));
      manipController.back().whileTrue(new TestSerializer(serializerSubsystem, -5));
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
  }

  public Command getAutonomousCommand() {
    // AUTOS not PATHS in path planner should be called here
    return new PathPlannerAuto("AutoName");
  }
}
