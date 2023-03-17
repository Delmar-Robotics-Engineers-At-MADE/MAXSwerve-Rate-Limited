// package frc.robot.Commands;

// import java.util.HashMap;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve.DriveSubsystem;

// public class DockAndEngageAuto {
//     // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// // for every path in the group
// DriveSubsystem m_robotDrive;

// PathPlannerTrajectory path = PathPlanner.loadPath("DockAndEngage", new PathConstraints(4, 3));

// // This is just an example event map. It would be better to have a constant, global event map
// // in your code that will be used by all path following commands.
// HashMap <String, Command> eventMap = new HashMap<>();
// eventMap.put("Balance", new Balance2());


// // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
// SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_robotDrive::getPose, m_robotDrive::resetOdometry, new PIDConstants(0.04, 0, 0), new PIDConstants(0.04, 0, 0), null, eventMap, m_robotDrive);

// Command fullAuto = autoBuilder.fullAuto(path);
// }
