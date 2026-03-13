package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import pabeles.concurrency.IntOperatorTask.Max;

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
    private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap(); // Meters: RPM
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap(); // Meters: degrees (double)
    private final InterpolatingDoubleTreeMap horizontalVelTable = new InterpolatingDoubleTreeMap(); // Meters: m/s
    private final InterpolatingDoubleTreeMap invHorizontalVelTable = new InterpolatingDoubleTreeMap(); //m/s: Meters


    public ShootOnMoveCmd(TurretSubsystem turretSubsystem, 
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

        for (Pair<Distance, AngularVelocity> entry : List.of(Pair.of(Meters.of(1), RPM.of((1000))),
                                    Pair.of(Meters.of(2), RPM.of(2000)),
                                    Pair.of(Meters.of(3), RPM.of(3000)))) {
            shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
        }
        for (Pair<Distance, Double> entry : List.of(Pair.of(Meters.of(1), 1.0),
                                    Pair.of(Meters.of(2), 1.0),
                                    Pair.of(Meters.of(3), 1.0))) {
            hoodTable.put(entry.getFirst().in(Meters), entry.getSecond());
        }
        for (Pair<Distance, Double> entry : List.of(Pair.of(Meters.of(1), 1.0),
                Pair.of(Meters.of(2), 1.0),
                Pair.of(Meters.of(3), 1.0))) {
                horizontalVelTable.put(entry.getFirst().in(Meters), entry.getSecond());
                invHorizontalVelTable.put(entry.getSecond(), entry.getFirst().in(Meters));
        }
    }

    public void initialize(){
        //put things that need to be initialized here (such as a timer). No need to @Override.
    }

    @Override
    public void execute() {
        var robotRelative = robotOrientedChassisSpeeds.get();
        var robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, robotPoseSupplier.get().getRotation());

        Translation2d futurePos = robotPoseSupplier.get().getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                    );

        // 2. GET TARGET VECTOR
        Translation2d goalLocation = goalPoseSupplier.get().getTranslation();
        Translation2d targetVec    = goalLocation.minus(futurePos);
        double        dist         = targetVec.getNorm();

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        double idealHorizontalSpeed = horizontalVelTable.get(dist);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngleDeg        = shotVec.getAngle().getDegrees();
        turretAngleDeg = Math.min(Math.max(turretAngleDeg, -99), 99); // Clamp turret degree
        Rotation2d turretAngle = Rotation2d.fromDegrees(turretAngleDeg); // New Rotation2d object using clamped angle
        double newHorizontalSpeed = shotVec.getNorm();

        double shotDist = invHorizontalVelTable.get(newHorizontalSpeed);
        if (shotDist < 4 * 0.3048){ // Can't shoot <4 ft
            return;
        }

        double exitRPM = shooterTable.get(shotDist);

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        // Clamp to avoid domain errors if we need more speed than possible
        // double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        // double newPitch = Math.acos(ratio);

        
        // 6.5. Correct field relative turret angle to robot relative (WITH 180 OFFSET)
        Rotation2d robotRelativeAngle = turretAngle
                                                    .minus(robotPoseSupplier.get().getRotation())
                                                    .minus(Rotation2d.fromDegrees(180));

        // 7. SET OUTPUTS
        turretSubsystem.setAngleDirect(Degrees.of(robotRelativeAngle.getDegrees()));
        // hoodSubsystem.setAngleDirect(Radians.of(newPitch));
        flywheelSubsystem.setVelocity(RPM.of(exitRPM));

        SmartDashboard.putBoolean("Target/canShoot", true);
        SmartDashboard.putNumber("Target/Turret Angle", robotRelativeAngle.getDegrees());
        SmartDashboard.putNumber("Target/RPM", exitRPM);
    }

    @Override
    public void end(boolean interrupted) {
        //this gets called when the input stops being given. 
        turretSubsystem.stop();
        hoodSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        //If true is returned, the command will stop being run. Can be used to check if a encoder is at right place or limit switch is press (for example)
        return false;
    }
}
