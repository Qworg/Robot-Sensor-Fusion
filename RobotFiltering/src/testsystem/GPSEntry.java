package testsystem;

//GPSEntry
/**
 * @author jeffreykramer
 *
 */
public class GPSEntry {
	/** DOCUMENT ME! */
	Double timestamp;

	Double NorthY;
	Double EastX;

	//GPS entries are in UTM set in Zone 17 in the Northern Hemisphere (meters)
	
	/**
	 * @param iEastX
	 * @param iNorthY
	 * @param timestamp
	 */
	public GPSEntry(Double iEastX, Double iNorthY, Double timestamp) {
		this.timestamp = timestamp;
		NorthY = iNorthY;
		EastX = iEastX;
	}
	
	/**
	 * @param iNorthY
	 */
	public void setNorthY(Double iNorthY) {
		NorthY = iNorthY;
	}

	/**
	 * @return
	 */
	public Double getNorthY() {
		return NorthY;
	}

	/**
	 * @param timestamp
	 */
	public void setGPSTimestamp(Double timestamp) {
		this.timestamp = timestamp;
	}

	/**
	 * @return
	 */
	public Double getGPSTimestamp() {
		return timestamp;
	}

	/**
	 * @param iEastX
	 */
	public void setEastX(Double iEastX) {
		EastX = iEastX;
	}

	
	/**
	 * @return
	 */
	public Double getEastX() {
		return EastX;
	}


	/**
	 * DOCUMENT ME!
	 * 
	 * @return DOCUMENT ME!
	 */
	public String toString() {
		StringBuffer sb = new StringBuffer();
		sb.append("Timestamp: ");
		sb.append(timestamp.toString());
		sb.append(")");
		sb.append(" Location: ");
		sb.append(EastX.toString() + " " + NorthY.toString());

		return sb.toString();
	}

//	private GMatrix getECEF() {
//		// Adapted from LLA2ECEF.m by Michael Kleder
//		// WGS84 ellipsoid constants
//		Double a = 6378137;
//		// Double e = 0.081819190842622;
//		// Other Variables
//		Double N, x, y, z, tLat, tLon, alt;
//		GMatrix mOutput = new GMatrix(1, 3);
//
//		alt = 0; // Altitude = 0?
//		// Calculate!
//
//		tLat = NorthX;
//		tLon = EastY;
//		// Intermediate Calc (prime vertical radius of curvature)
//		N = a / Math.sqrt(1 - (Math.E * Math.E) * Math.sin(tLat));
//
//		// Results
//		x = (N + alt) * Math.cos(tLat) * Math.cos(tLon);
//		y = (N + alt) * Math.cos(tLat) * Math.sin(tLon);
//		z = ((1 - (Math.exp(Math.E))) * N + alt) * Math.sin(tLat);
//
//		// Put it in a Matrix
//		mOutput.setElement(0, 0, x);
//		mOutput.setElement(0, 1, y);
//		mOutput.setElement(0, 2, z);
//
//		return mOutput;
//	}

}
