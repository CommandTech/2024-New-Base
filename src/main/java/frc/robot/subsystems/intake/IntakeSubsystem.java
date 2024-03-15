// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static class Hardware {
    private CANSparkMax rollerMotor;

    public Hardware(CANSparkMax rollerMotor) {
      this.rollerMotor = rollerMotor;
    }
  }

  private CANSparkMax m_rollerMotor;

  private final Measure<Dimensionless> ROLLER_VELOCITY;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardware, Measure<Dimensionless> rollerVelocity) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
    ROLLER_VELOCITY = rollerVelocity;
    this.m_rollerMotor = intakeHardware.rollerMotor;

    // Set idle mode
    m_rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Initialize hardware devices for intake subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new CANSparkMax(Constants.CANIDConstants.kIntakeCandId, MotorType.kBrushless)
    );
    return intakeHardware;
  }

  // Tells the robot to intake
  private void intake() {
    m_rollerMotor.set(-ROLLER_VELOCITY.in(Units.Percent));
  }

  // Tells the robot to outtake
  private void outtake() {
    m_rollerMotor.set(ROLLER_VELOCITY.in(Units.Percent));
  }

  // Turn outtake on
  private void toggleOuttakeOn() {
    m_rollerMotor.set(ROLLER_VELOCITY.in(Units.Percent));
  }

  // Stop the robot
  private void stop() {
    m_rollerMotor.stopMotor();;
  }

  @Override
  public void periodic() {

  }

  /**
   * Intake game piece from ground
   * @return Command to run the roller motor
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Spit out game piece from intake
   * @return Command to run the roller motor in the reverse direction
   */
  public Command outtakeCommand() {
    return startEnd(() -> outtake(), () -> stop());
  }

  /**
   * Spit out game piece from intake
   * @return Command to run the roller motor in the reverse direction
   */
  public Command toggleOuttakeOnCommand() {
    return runOnce(() -> toggleOuttakeOn());
  }

  public Command stopCommand() {
    return runOnce(() -> stop());  
}
}