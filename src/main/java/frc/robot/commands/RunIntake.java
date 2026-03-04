package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {

  private final Command internalCommand;

  public RunIntake(IntakeSubsystem intakeSubsystem, double armIntakeTargetAngle) {

    Angle angle = Degrees.of(armIntakeTargetAngle);

    internalCommand = intakeSubsystem.intake(angle);

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    internalCommand.initialize();
  }

  @Override
  public void execute() {
    internalCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    internalCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return internalCommand.isFinished();
  }
}
