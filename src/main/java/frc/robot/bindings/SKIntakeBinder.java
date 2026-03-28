package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.kIntakeRollersVoltage;
import static frc.robot.Ports.DriverPorts.kLTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.Intake.SKIntakePivot;
import frc.robot.subsystems.Intake.SKIntakeRollers;

public class SKIntakeBinder implements CommandBinder {

    Optional<SKIntakeRollers> m_intakeRollers;
    Optional<SKIntakePivot> m_intakePivot;

    Trigger intake;

    public SKIntakeBinder(Optional<SKIntakeRollers> intakeRollers, Optional<SKIntakePivot> intakePivot) {

        m_intakeRollers = intakeRollers;
        m_intakePivot = intakePivot;

        intake = kLTrigger.button;
    }

    @Override
    public void bindButtons() {
        // TODO Auto-generated method stub
        if(m_intakeRollers.isPresent()) {
            SKIntakeRollers intakeRollers = m_intakeRollers.get();
            intake.whileTrue(intakeRollers.runIntakeRollersCommand(kIntakeRollersVoltage));
        }
        if(m_intakePivot.isPresent()) {
            SKIntakePivot intakePivot = m_intakePivot.get();
            intake.onTrue(intakePivot.deployIntakeCommand());
        }
    }
    
}
