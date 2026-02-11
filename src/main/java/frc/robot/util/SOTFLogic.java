package frc.robot.util; // or wherever you want

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.subsystems.Shooter.Shotmap.State;

public final class SOTFLogic {

    private SOTFLogic() {} // prevent instantiation

    public static Shotmap.State SOTFLogic(
        Pose2d robotPose,
        Translation2d hubPos,
        Translation2d robotFieldVelocity
    ) {
        Distance dist = Shotmap.distanceToHub(robotPose, hubPos);
        Shotmap.State base = Shotmap.getState(dist);

        Translation2d toHub =
            hubPos.minus(robotPose.getTranslation()).div(dist.in(Meters));

        double vHorizIdeal =
            Shotmap.getHorizontalVelocity(dist).in(MetersPerSecond);

        Translation2d vTarget = toHub.times(vHorizIdeal);
        Translation2d vShot = vTarget.minus(robotFieldVelocity);

        double newHorizSpeed = vShot.getNorm();

        double vTotal = Shotmap.rpmToLinear(base.getVelocity());
        double vVert = vTotal * Math.sin(base.getAngle().in(Radians));

        double newPitch = Math.atan2(vVert, newHorizSpeed);

        return new Shotmap.State(
            Radians.of(newPitch),
            Shotmap.linearToRPM(Math.hypot(newHorizSpeed, vVert)),
            base.getTof(),
            base.getHeight()
        );
    }
}
