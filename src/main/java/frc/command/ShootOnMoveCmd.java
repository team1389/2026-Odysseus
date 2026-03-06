package frc.command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.subsystems.FlywheelSubsystem;
import frc.subsystems.HoodSubsystem;
import frc.subsystems.TurretSubsystem;
import java.util.List;
import java.util.function.Supplier;

public class ShootOnMoveCmd extends Command {
    private final TurretSubsystem turretSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
    private final Pose2d goalPose;
    double totalExitVelocity = 15.0; // m/s
    private final double latency = 0.15;
    private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap(); // degrees


    public ShootOnMoveCmd(TurretSubsystem turretSubsystem, 
                            FlywheelSubsystem flywheelSubsystem,
                            HoodSubsystem hoodSubsystem,
                            Supplier<Pose2d> robotPose,
                            Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                            Pose2d goalPose) {
        this.turretSubsystem = turretSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.robotPose = robotPose;
        this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
        this.goalPose = goalPose;

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
    }

    public void initialize(){
        //put things that need to be initialized here (such as a timer). No need to @Override.
    }

    @Override
    public void execute() {
        var robotSpeed = fieldOrientedChassisSpeeds.get();
        Translation2d futurePos = robotPose.get().getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                    );

        // 2. GET TARGET VECTOR
        Translation2d goalLocation = goalPose.getTranslation();
        Translation2d targetVec    = goalLocation.minus(futurePos);
        double        dist         = targetVec.getNorm();

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        double idealHorizontalSpeed = shooterTable.get(dist);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngle        = shotVec.getAngle().getDegrees();
        double newHorizontalSpeed = shotVec.getNorm();

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        // 7. SET OUTPUTS
        turretSubsystem.setAngleDirect(Degrees.of(turretAngle));
        hoodSubsystem.setAngleDirect(Radians.of(newPitch));
        flywheelSubsystem.setRPMDirect(MetersPerSecond.of(totalExitVelocity));
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
