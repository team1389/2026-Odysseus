package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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
import frc.robot.util.AllianceFlipUtil;
import java.util.List;
import java.util.function.Supplier;

public class PassingModeCmd extends Command {
  private final TurretSubsystem turretSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotOrientedChassisSpeeds;

  public Pose2d LeftPose() {
    return AllianceFlipUtil.flip(new Pose2d(Inches.of(30), Inches.of(130), Rotation2d.kZero));
  }

  public Pose2d RightPose() {
    return AllianceFlipUtil.flip(new Pose2d(Inches.of(290), Inches.of(130), Rotation2d.kZero));
  }

  private final double latency = 0.15; //      <---------------NEED TO TUNE
  private final InterpolatingDoubleTreeMap shooterTable =
      new InterpolatingDoubleTreeMap(); // Meters: RPM
  private final InterpolatingDoubleTreeMap horizontalVelTable =
      new InterpolatingDoubleTreeMap(); // Meters: m/s
  private final InterpolatingDoubleTreeMap invHorizontalVelTable =
      new InterpolatingDoubleTreeMap(); // m/s: Meters

  public PassingModeCmd(
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

    for (Pair<Distance, AngularVelocity> entry :
        List.of(
            Pair.of(Meters.of(0.0), RPM.of((0))),
            Pair.of(Meters.of(7 * 0.3048), RPM.of(796)),
            Pair.of(Meters.of(8 * 0.3048), RPM.of(832)),
            Pair.of(Meters.of(9 * 0.3048), RPM.of(892)),
            Pair.of(Meters.of(10 * 0.3048), RPM.of(948)),
            Pair.of(Meters.of(11 * 0.3048), RPM.of(1001)),
            Pair.of(Meters.of(12 * 0.3048), RPM.of(1051)),
            Pair.of(Meters.of(13 * 0.3048), RPM.of(1100)),
            Pair.of(Meters.of(14 * 0.3048), RPM.of(1146)),
            Pair.of(Meters.of(15 * 0.3048), RPM.of(1191)),
            Pair.of(Meters.of(16 * 0.3048), RPM.of(1234)),
            Pair.of(Meters.of(17 * 0.3048), RPM.of(1275)),
            Pair.of(Meters.of(18 * 0.3048), RPM.of(1316)),
            Pair.of(Meters.of(19 * 0.3048), RPM.of(1355)),
            Pair.of(Meters.of(20 * 0.3048), RPM.of(1393)),
            Pair.of(Meters.of(21 * 0.3048), RPM.of(1430)),
            Pair.of(Meters.of(22 * 0.3048), RPM.of(1430)),
            Pair.of(Meters.of(23 * 0.3048), RPM.of(1467)),
            Pair.of(Meters.of(24 * 0.3048), RPM.of(1537)),
            Pair.of(Meters.of(25 * 0.3048), RPM.of(1570)),
            Pair.of(Meters.of(26 * 0.3048), RPM.of(1604)),
            Pair.of(Meters.of(27 * 0.3048), RPM.of(1636)),
            Pair.of(Meters.of(28 * 0.3048), RPM.of(1668)),
            Pair.of(Meters.of(29 * 0.3048), RPM.of(1699)),
            Pair.of(Meters.of(30 * 0.3048), RPM.of(1730)),
            Pair.of(Meters.of(31 * 0.3048), RPM.of(1760)),
            Pair.of(Meters.of(32 * 0.3048), RPM.of(1790)),
            Pair.of(Meters.of(33 * 0.3048), RPM.of(1819)),
            Pair.of(Meters.of(34 * 0.3048), RPM.of(1848)),
            Pair.of(Meters.of(35 * 0.3048), RPM.of(1876)),
            Pair.of(Meters.of(36 * 0.3048), RPM.of(1904)),
            Pair.of(Meters.of(37 * 0.3048), RPM.of(1931)),
            Pair.of(Meters.of(38 * 0.3048), RPM.of(1958)),
            Pair.of(Meters.of(39 * 0.3048), RPM.of(1985)),
            Pair.of(Meters.of(40 * 0.3048), RPM.of(2011)),
            Pair.of(Meters.of(41 * 0.3048), RPM.of(2037)),
            Pair.of(Meters.of(42 * 0.3048), RPM.of(2063)),
            Pair.of(Meters.of(43 * 0.3048), RPM.of(2089)),
            Pair.of(Meters.of(44 * 0.3048), RPM.of(2114)),
            Pair.of(Meters.of(45 * 0.3048), RPM.of(2138)),
            Pair.of(Meters.of(46 * 0.3048), RPM.of(2163)),
            Pair.of(Meters.of(47 * 0.3048), RPM.of(2187)),
            Pair.of(Meters.of(48 * 0.3048), RPM.of(2211)),
            Pair.of(Meters.of(49 * 0.3048), RPM.of(2235)),
            Pair.of(Meters.of(50 * 0.3048), RPM.of(2258)))) {
      shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
    }

    for (Pair<Distance, Double> entry :
        List.of(
            Pair.of(Meters.of(0.0), 0.0),
            Pair.of(Meters.of((4 + 1.95833) * 0.3048), 5.0),
            Pair.of(Meters.of((5 + 1.95833) * 0.3048), 5.53097),
            Pair.of(Meters.of((6 + 1.95833) * 0.3048), 6.60793),
            Pair.of(Meters.of((7 + 1.95833) * 0.3048), 6.73077),
            Pair.of(Meters.of((8 + 1.95833) * 0.3048), 7.25953),
            Pair.of(Meters.of((9 + 1.95833) * 0.3048), 8.09353),
            Pair.of(Meters.of((10 + 1.95833) * 0.3048), 8.15661),
            Pair.of(Meters.of((11 + 1.95833) * 0.3048), 8.94309),
            Pair.of(Meters.of((12 + 1.95833) * 0.3048), 10.27397),
            Pair.of(Meters.of((13 + 1.95833) * 0.3048), 10.14041),
            Pair.of(Meters.of((14 + 1.95833) * 0.3048), 10.63830),
            Pair.of(Meters.of((15 + 1.95833) * 0.3048), 12.11632),
            Pair.of(Meters.of((16 + 1.95833) * 0.3048), 12.08459),
            Pair.of(Meters.of((17 + 1.95833) * 0.3048), 12.33672),
            Pair.of(Meters.of((18 + 1.95833) * 0.3048), 13.45291),
            Pair.of(Meters.of((19 + 1.95833) * 0.3048), 12.59947),
            Pair.of(Meters.of((20 + 1.95833) * 0.3048), 13.58696))) {
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
    Translation2d targetVec = LeftPose().getTranslation().minus(futurePos);
    if (futurePos.getX() > 4) {
      targetVec = RightPose().getTranslation().minus(futurePos);
    }
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
    if (shotDist < (4 + 1.95833) * 0.3048) { // Can't shoot <4ft
      SmartDashboard.putBoolean("Target/canShoot", false);
      return;
    }

    double exitRPM = shooterTable.get(shotDist);

    // Correct field relative turret angle to robot relative (WITH 180 OFFSET)
    Rotation2d robotRelativeAngle =
        turretAngle.minus(robotPoseSupplier.get().getRotation()).minus(Rotation2d.fromDegrees(180));

    double hoodAngle = 45;

    // SET OUTPUTS
    turretSubsystem.setAngleDirect(Degrees.of(robotRelativeAngle.getDegrees()));
    hoodSubsystem.setAngleDirect(Degrees.of(hoodAngle));
    flywheelSubsystem.setVelocity(RPM.of(exitRPM));

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
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
