package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.List;
import java.util.function.Supplier;

public class ShootOnMoveCmd extends Command {
  private final TurretSubsystem turretSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotOrientedChassisSpeeds;
  private final Supplier<Pose2d> goalPoseSupplier;
  private final double latency = 0.15; //      <---------------NEED TO TUNE
  private final double offset =
      0.25; //      <---------------NEED TO TUNE (in feet, added to distance to target for
  // interpolation tables to account for the fact that the robot is moving towards the target
  // while shooting)
  private final InterpolatingDoubleTreeMap shooterTable =
      new InterpolatingDoubleTreeMap(); // Meters: RPM
  private final InterpolatingDoubleTreeMap hoodTable =
      new InterpolatingDoubleTreeMap(); // Meters: degrees (double)
  private final InterpolatingDoubleTreeMap horizontalVelTable =
      new InterpolatingDoubleTreeMap(); // Meters: m/s
  private final InterpolatingDoubleTreeMap invHorizontalVelTable =
      new InterpolatingDoubleTreeMap(); // m/s: Meters

  public ShootOnMoveCmd(
      TurretSubsystem turretSubsystem,
      FlywheelSubsystem flywheelSubsystem,
      HoodSubsystem hoodSubsystem,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotOrientedChassisSpeeds,
      Supplier<Pose2d> goalPoseSupplier) {
    this.turretSubsystem = turretSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotOrientedChassisSpeeds = robotOrientedChassisSpeeds;
    this.goalPoseSupplier = goalPoseSupplier;
    addRequirements(turretSubsystem, hoodSubsystem, flywheelSubsystem);
    for (Pair<Distance, AngularVelocity> entry :
        List.of(
            Pair.of(Meters.of(0.0), RPM.of((0))),
            Pair.of(Meters.of((4 + offset) * 0.3048), RPM.of((1600))),
            Pair.of(Meters.of((5 + offset) * 0.3048), RPM.of(1700)),
            Pair.of(Meters.of((6 + offset) * 0.3048), RPM.of(1775)),
            Pair.of(Meters.of((7 + offset) * 0.3048), RPM.of(1850)),
            Pair.of(Meters.of((8 + offset) * 0.3048), RPM.of(1890)),
            Pair.of(Meters.of((9 + offset) * 0.3048), RPM.of(2000)),
            Pair.of(Meters.of((10 + offset) * 0.3048), RPM.of(2026)),
            Pair.of(Meters.of((11 + offset) * 0.3048), RPM.of(2100)),
            Pair.of(Meters.of((12 + offset) * 0.3048), RPM.of(2050)),
            Pair.of(Meters.of((13 + offset) * 0.3048), RPM.of(2100)),
            Pair.of(Meters.of((14 + offset) * 0.3048), RPM.of(2150)),
            Pair.of(Meters.of((15 + offset) * 0.3048), RPM.of(2150)),
            Pair.of(Meters.of((16 + offset) * 0.3048), RPM.of(2175)),
            Pair.of(Meters.of((17 + offset) * 0.3048), RPM.of(2270)),
            Pair.of(Meters.of((18 + offset) * 0.3048), RPM.of(2375)),
            Pair.of(Meters.of((19 + offset) * 0.3048), RPM.of(2400)),
            Pair.of(Meters.of((20 + offset) * 0.3048), RPM.of(2400)))) {
      shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
    }
    for (Pair<Distance, Double> entry :
        List.of(
            Pair.of(Meters.of((4 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((5 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((6 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((7 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((8 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((9 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((10 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((11 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((12 + offset) * 0.3048), 0.0),
            Pair.of(Meters.of((13 + offset) * 0.3048), 5.0),
            Pair.of(Meters.of((14 + offset) * 0.3048), 5.0),
            Pair.of(Meters.of((15 + offset) * 0.3048), 5.5),
            Pair.of(Meters.of((16 + offset) * 0.3048), 5.5),
            Pair.of(Meters.of((17 + offset) * 0.3048), 6.0),
            Pair.of(Meters.of((18 + offset) * 0.3048), 6.0),
            Pair.of(Meters.of((19 + offset) * 0.3048), 8.0),
            Pair.of(Meters.of((20 + offset) * 0.3048), 10.0))) {
      hoodTable.put(entry.getFirst().in(Meters), entry.getSecond());
    }
    for (Pair<Distance, Double> entry :
        List.of(
            Pair.of(Meters.of(0.0), 0.0),
            Pair.of(Meters.of((4 + offset) * 0.3048), 5.0),
            Pair.of(Meters.of((5 + offset) * 0.3048), 5.53097),
            Pair.of(Meters.of((6 + offset) * 0.3048), 6.60793),
            Pair.of(Meters.of((7 + offset) * 0.3048), 6.73077),
            Pair.of(Meters.of((8 + offset) * 0.3048), 7.25953),
            Pair.of(Meters.of((9 + offset) * 0.3048), 8.09353),
            Pair.of(Meters.of((10 + offset) * 0.3048), 8.15661),
            Pair.of(Meters.of((11 + offset) * 0.3048), 8.94309),
            Pair.of(Meters.of((12 + offset) * 0.3048), 10.27397),
            Pair.of(Meters.of((13 + offset) * 0.3048), 10.14041),
            Pair.of(Meters.of((14 + offset) * 0.3048), 10.63830),
            Pair.of(Meters.of((15 + offset) * 0.3048), 12.11632),
            Pair.of(Meters.of((16 + offset) * 0.3048), 12.08459),
            Pair.of(Meters.of((17 + offset) * 0.3048), 12.33672),
            Pair.of(Meters.of((18 + offset) * 0.3048), 13.45291),
            Pair.of(Meters.of((19 + offset) * 0.3048), 12.59947),
            Pair.of(Meters.of((20 + offset) * 0.3048), 13.58696))) {
      horizontalVelTable.put(entry.getFirst().in(Meters), entry.getSecond());
      invHorizontalVelTable.put(entry.getSecond(), entry.getFirst().in(Meters));
    }
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    var robotRelative = robotOrientedChassisSpeeds.get();
    var robotSpeed =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, robotPoseSupplier.get().getRotation());

    Translation2d futurePos =
        robotPoseSupplier
            .get()
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
                    .times(latency));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPoseSupplier.get().getTranslation();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = horizontalVelTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngleDeg = shotVec.getAngle().getDegrees();
    Rotation2d turretAngle =
        Rotation2d.fromDegrees(turretAngleDeg); // New Rotation2d object using clamped angle
    double newHorizontalSpeed = shotVec.getNorm();

    double shotDist = invHorizontalVelTable.get(newHorizontalSpeed);
    if (shotDist < (4 + offset) * 0.3048) { // Can't shoot <4ft
      SmartDashboard.putBoolean("Target/canShoot", false);
      return;
    }

    double exitRPM = shooterTable.get(shotDist);

    // Correct field relative turret angle to robot relative (WITH 180 OFFSET)
    Rotation2d robotRelativeAngle =
        turretAngle.minus(robotPoseSupplier.get().getRotation()).minus(Rotation2d.fromDegrees(180));

    double hoodAngle = hoodTable.get(shotDist);

    // SET OUTPUTS
    turretSubsystem.setAngleDirect(Degrees.of(robotRelativeAngle.getDegrees()));
    hoodSubsystem.setAngleDirect(Degrees.of(hoodAngle));
    flywheelSubsystem.setRPM(RPM.of(exitRPM));

    SmartDashboard.putBoolean("Target/canShoot", true);
    SmartDashboard.putNumber("Target/Turret Angle", robotRelativeAngle.getDegrees());
    SmartDashboard.putNumber("Target/Hood Angle", hoodAngle);
    SmartDashboard.putNumber("Target/RPM", exitRPM);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    turretSubsystem.stop();
    hoodSubsystem.stop();
    flywheelSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
