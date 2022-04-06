package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    //main network table
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    //distance stuff
    private double mountAngle;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

    public Limelight(double mountAngle) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        this.mountAngle = mountAngle;
    }

    public double getX() {
        return tx.getDouble(0);
    }

    public double getY() {
        return ty.getDouble(0);
    }

    public double getAngle() {
        return ta.getDouble(0);
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0) == 1 ? true : false;
    }

    public void setLEDState(int state) {
        table.getEntry("ledMode").setNumber(state);
    }

    public double getDistanceFromTarget(double goalHeight, double camHeight) {
        double distance = 0;
        if (!hasValidTarget()) {
            distance = -1;
        }
        else {
            distance = (goalHeight - camHeight) / Math.tan((mountAngle + ty.getDouble(0)) * (Math.PI / 180));
        }
        return distance;
    }

    //-right +left
    public double getAlignmentAdjustment() {
        double min = 0.05; //tune
        double adjustment = 0;
        double kP = 0.1; //tune
        double tx_angle = tx.getDouble(0);
        double headingErr = -tx_angle;
        if (hasValidTarget()) {
            if (tx_angle > 1) {
                adjustment = kP * headingErr - min;
            }
            else if (tx_angle < 1) {
                adjustment = kP * headingErr + min;
            }
        }
        return adjustment;
    }
}