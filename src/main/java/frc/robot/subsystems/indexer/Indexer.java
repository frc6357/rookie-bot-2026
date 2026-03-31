package frc.robot.subsystems.indexer;

/* NOTE: This code and these comments were made by an inexperienced rookie.
 * Tread carefuly.
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// imports the variable for the motor port
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

// imports the class that helps control the motor
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Indexer extends SubsystemBase {
    // motor initialization
    SparkFlex indexerMotor = new SparkFlex(kIndexerMotor.ID, MotorType.kBrushless);

    Indexer() {
       SparkFlexConfig config = new SparkFlexConfig();
    }

    @Override
    public void periodic() {

    }
    
}
