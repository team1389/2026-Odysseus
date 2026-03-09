package frc.robot.commands.AutoCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeArm extends Command {
  public IntakeSubsystem intakeSubsystem;
  public double targetAngle = RobotMap.IntakeArmAngle;

  public MoveIntakeArm(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    intakeSubsystem.setAngle(Degrees.of(targetAngle));
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    intakeSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
