package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake {
    public WPI_VictorSPX intake;

    public Intake(){
        intake = new WPI_VictorSPX(Constants.intake_port);
    }

    public void set_speed(double speed){
        intake.set(speed);
    }

    public void stop(){
        intake.set(0);
    }
}
