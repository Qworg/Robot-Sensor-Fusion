package testsystem;

public class OdoEntry {

	double X, Y, theta;
	double timestamp;

	public OdoEntry(double d_X, double d_Y, double d_Theta, double timestamp) {
		this.X = d_X;
		this.Y = d_Y;
		this.theta = d_Theta;
		this.timestamp = timestamp;
	}
	
	public void setOdoX(double d_X) {
		this.X = d_X;
	}
	public void setOdoY(double d_Y) {
		this.Y = d_Y;
	}
	public void setOdoTheta(double d_Theta) {
		this.theta = d_Theta;
	}
	
	public double getOdoX() {
		return X;
	}
	public double getOdoY() {
		return Y;
	}
	public double getOdoTheta() {
		return theta;
	}
	
	public void setOdoTimestamp(double timestamp){
		this.timestamp = timestamp;
	}
	public double getOdoTimestamp(){
		return timestamp;
	}
	
	public OdoEntry CopyOdoEntry() {
		OdoEntry retval = new OdoEntry(X, Y, theta, timestamp);
		return retval;
	}

}
