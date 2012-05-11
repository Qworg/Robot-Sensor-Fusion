package testsystem;

public class CompassEntry {

	Double Theta;
	Double dTS;

	public CompassEntry(Double Heading, Double Timestamp) {
		//Make Heading into Rads - Theta is stored as Rads
		Theta = Heading*(Math.PI/180);
		dTS = Timestamp;
	}
	
	public CompassEntry(Double Heading, Double Timestamp, boolean Rads) {
		//Check if Rads, then set appropriately
		dTS = Timestamp;
		if (Rads)
			Theta = Heading;
		else
			Theta = Heading*(Math.PI/180);
	}
	
	public Double GetDeg(){
		return (Theta*180)/Math.PI;
	}
	
	public Double GetRad(){
		return Theta;
	}
	
	public void SetDeg(Double Heading){
		Theta = Heading*(Math.PI/180);
	}
	
	public void SetRad(Double Heading){
		Theta = Heading;
	}
	
	public Double GetTimestep(){
		return dTS;
	}
	
	public void SetTimestep(Double Timestamp){
		dTS = Timestamp;
	}
	
	public CompassEntry CopyCompassEntry() {
		CompassEntry retval = new CompassEntry(Theta*(180/Math.PI), dTS);
		return retval;
	}

}
