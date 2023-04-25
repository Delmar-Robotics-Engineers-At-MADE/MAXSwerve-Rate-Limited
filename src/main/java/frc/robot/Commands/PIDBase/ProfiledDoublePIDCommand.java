// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PIDBase;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


/**
 * A command that controls an output with a {@link ProfiledPIDController}. Runs forever by default -
 * to add exit conditions and/or other behavior, subclass this class. The controller calculation and
 * output are performed synchronously in the command's execute() method.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class ProfiledDoublePIDCommand extends CommandBase {

    public static class DoubleDouble {
        public double x1;
        public double x2;

        public DoubleDouble() {}

        public DoubleDouble(double first, double second) {
            this.x1 = first;
            this.x2 = second;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof DoubleDouble) {
                DoubleDouble rhs = (DoubleDouble) other;
                return this.x1 == rhs.x1 && this.x2 == rhs.x2;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(x1, x2);
        }
    }

    public static class DoubleState {
        public State s1;
        public State s2;

        public DoubleState() {}

        public DoubleState(State first, State second) {
            this.s1 = first;
            this.s2 = second;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof DoubleState) {
                DoubleState rhs = (DoubleState) other;
                return this.s1.equals(rhs.s1) && this.s2.equals(rhs.s2);
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(s1.hashCode(), s2.hashCode());
        }
    }




    protected final ProfiledPIDController m_controller;
    protected DoubleSupplier m_measurement;
    protected Supplier<State> m_goal;
    protected final ProfiledPIDController m_controller2;
    protected DoubleSupplier m_measurement2;
    protected Supplier<State> m_goal2;
    protected BiConsumer<DoubleDouble, DoubleState> m_useOutput;

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is specified.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goalSource the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
//   public ProfiledDoublePIDCommand(
//       ProfiledPIDController controller,
//       DoubleSupplier measurementSource,
//       Supplier<State> goalSource,
//       BiConsumer<Double, State> useOutput,
//       Subsystem... requirements) {
//     requireNonNullParam(controller, "controller", "ProfiledPIDCommand");
//     requireNonNullParam(measurementSource, "measurementSource", "ProfiledPIDCommand");
//     requireNonNullParam(goalSource, "goalSource", "ProfiledPIDCommand");
//     requireNonNullParam(useOutput, "useOutput", "ProfiledPIDCommand");

//     m_controller = controller;
//     m_useOutput = useOutput;
//     m_measurement = measurementSource;
//     m_goal = goalSource;
//     m_requirements.addAll(Set.of(requirements));
//   }

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is implicitly zero.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goalSource the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  public ProfiledDoublePIDCommand(
    ProfiledPIDController controller,
    ProfiledPIDController controller2,
    DoubleSupplier measurementSource,
    DoubleSupplier measurementSource2,
    DoubleSupplier goalSource,
    DoubleSupplier goalSource2,
    BiConsumer<DoubleDouble, DoubleState> useOutput,
      Subsystem... requirements) {
    requireNonNullParam(controller, "controller", "SynchronousPIDCommand");
    requireNonNullParam(measurementSource, "measurementSource", "SynchronousPIDCommand");
    requireNonNullParam(goalSource, "goalSource", "SynchronousPIDCommand");
    requireNonNullParam(useOutput, "useOutput", "SynchronousPIDCommand");

    m_controller = controller;
    m_useOutput = useOutput;
    m_measurement = measurementSource;
    m_goal = () -> new State(goalSource.getAsDouble(), 0);
    m_controller2 = controller2;
    m_measurement2 = measurementSource2;
    m_goal2 = () -> new State(goalSource2.getAsDouble(), 0);
    m_requirements.addAll(Set.of(requirements));
  }

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is implicitly zero.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goal the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  public ProfiledDoublePIDCommand(
    ProfiledPIDController controller, ProfiledPIDController controller2,
    DoubleSupplier measurementSource, DoubleSupplier measurementSource2,
    double goal, double goal2,
      BiConsumer<DoubleDouble, DoubleState> useOutput,
      Subsystem... requirements) {
        this(controller, controller2, measurementSource, measurementSource2, 
             () -> goal, () -> goal2, useOutput, requirements);
  }

  @Override
  public void initialize() {
    m_controller.reset(m_measurement.getAsDouble());
    m_controller2.reset(m_measurement2.getAsDouble());
  }

  @Override
  public void execute() {
    m_useOutput.accept(
        new DoubleDouble (m_controller.calculate(m_measurement.getAsDouble(), m_goal.get()), 
                          m_controller2.calculate(m_measurement2.getAsDouble(), m_goal2.get())),
        new DoubleState(m_controller.getSetpoint(), m_controller2.getSetpoint()));
  }

  @Override
  public void end(boolean interrupted) {
    m_useOutput.accept(new DoubleDouble(0,0), new DoubleState(new State(), new State()));
  }

  /**
   * Returns the ProfiledPIDController used by the command.
   *
   * @return The ProfiledPIDController
   */
  public ProfiledPIDController getController1() {
    return m_controller;
  }
  public ProfiledPIDController getController2() {
    return m_controller2;
  }
}
