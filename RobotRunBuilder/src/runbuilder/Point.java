package runbuilder;

public class Point {
	private Double x; // x-coordinate
	private Double y; // y-coordinate

	public Point() {
		//From 0 to 1
		x = Math.random();
		y = Math.random();
	}

	public Point(Double x, Double y) {
		//Point created from #s
		this.x = x;
		this.y = y;
	}

	//  Returns
	public Double getX() {
		return x;
	}

	public Double getY() {
		return y;
	}

	public Double getR() {
		return Math.sqrt(x * x + y * y);
	}

	public Double getTheta() {
		return Math.atan2(y, x);
	}
	
	public Double getTheta(Point that) {
		Double dx = that.x - this.x;
		Double dy = that.y - this.y;
		return Math.atan2(dy, dx);
	}

	// Euclidean distance between this point and that point
	public Double distTo(Point that) {
		Double dx = this.x - that.x;
		Double dy = this.y - that.y;
		return Math.sqrt(dx * dx + dy * dy);
	}

	public String toString() {
		// return a string representation of this point
		return x + "," + y;
	}

}
