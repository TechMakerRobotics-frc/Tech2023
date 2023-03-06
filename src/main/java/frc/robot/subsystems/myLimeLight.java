package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelightvision.limelight.frc.Limelight;


/**
 *
 */
public class myLimeLight extends SubsystemBase {

    private Limelight _limelight;

    public myLimeLight() {
        _limelight = new Limelight();
        //_limelight = new LimeLight("limelight");
    }


    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public Limelight getLimeLight(){
        return _limelight;
    }
   
}
