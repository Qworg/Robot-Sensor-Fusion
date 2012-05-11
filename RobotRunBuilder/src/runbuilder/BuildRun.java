package runbuilder;

import java.io.*;
import java.util.Random;
import java.util.Vector;

public class BuildRun {

	/**
	 * @param args
	 */

	// Args:
	// 0 - Output File Name (will add .txt)
	// 1 - Microseconds of run duration OR Input File Name OR "Random" (without quotes)
	// 2 - (0 or 1) GPS malfunction (will add random GPS malfunction)
	// 3 - (0 or 1) IMU malfunction (will add random IMU malfunction)
	// 4 - (0 or 1) Compass malfunction (will add random compass malfunction)
	// 5 - (0 or 1) Odo Malfunction (will add random Odo Malfunction)
	public static void main(String[] args) {
		boolean GPSMal = false;
		boolean IMUMal = false;
		boolean CompassMal = false;
		boolean OdoMal = false;
		boolean GPSBridge = false;
		boolean Simple = true;
		boolean Random = false;
		Integer RunDuration;
		Vector<Point> Points = new Vector<Point>();
		String CheckDown = args[1];
		if (CheckDown.contains(".csv")) {
			RunDuration = 0;
			Simple = false;
			Random = false;
		} else if (CheckDown.equalsIgnoreCase("Random")) {
			RunDuration = 0;
			Simple = false;
			Random = true;
		} else {
			Simple = true;
			Random = false;
			RunDuration = Integer.valueOf(CheckDown);
		}
		String DataOut = args[0];

		// System.out.println(args[0]);
		// System.out.println(args[1]);
		// System.out.println(args[2]);
		// System.out.println(args[3]);
		// System.out.println(args[4]);
		// System.out.println(args[5]);
		// System.out.println(args[6]);
		if (args[2].equals("1"))
			GPSMal = true;
		if (args[3].equals("1"))
			GPSBridge = true;
		GPSBridge = true;
		if (args[4].equals("1"))
			IMUMal = true;
		if (args[5].equals("1"))
			CompassMal = true;
		if (args[6].equals("1"))
			OdoMal = true;

		if (!Simple)
			if (Random) {
				Random r = new Random();
				// Generate a random number of points from 15-20
				//int j = r.nextInt(6) + 15;
				int j = 20;
				System.out.println("Number of Random Points: " + j);
				for (int i = 0; i < j; i++) {
					Points.add(new Point(1000.0 * r.nextDouble(), 1000.0 * r.nextDouble()));
					System.out.println(Points.get(Points.size()-1).toString());
				}
			} else {
				try {
					BufferedReader pin = new BufferedReader(new FileReader(CheckDown));
					pin.readLine();
					String Line;
					int NumPoints = 0;
					while ((Line = pin.readLine()) != null) {
						Line = Line.trim();
						Points.add(new Point(Double.valueOf(Line.substring(0, Line.indexOf(",")).trim()), Double.valueOf(Line.substring(Line.indexOf(",")+1).trim())));
						System.out.println("Number of Points: " + NumPoints++);
					}
					
				} catch (FileNotFoundException e) {
					// TODO Auto-generated catch block
					System.out.println("No Such Input File");
					e.printStackTrace();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					System.out.println("I/O Error on Input");
					e.printStackTrace();
				}

			}

		String ControlOut = DataOut + "Control.txt";
		DataOut = DataOut + ".txt";

		RunMaker run;

		if (Simple)
			run = new RunMaker(GPSMal, IMUMal, CompassMal, OdoMal, GPSBridge, RunDuration);
		else
			run = new RunMaker(GPSMal, IMUMal, CompassMal, OdoMal, GPSBridge, Points);

		try {
			BufferedWriter dout = new BufferedWriter(new FileWriter(DataOut));
			BufferedWriter cout = new BufferedWriter(new FileWriter(ControlOut));
			dout.write("#timestamp	elapsedtime	IMUAccelX	IMUAccelY	IMUAccelZ	HeadingX	HeadingY	HeadingActual	unused	unused	unused	XOdo	YOdo	Theta	GPSY	GPSX");
			dout.newLine();
			// cout.write("timestamp,elapsedtime,ActualX,ActualY,ActualHeading");
			cout.write("#timestamp	elapsedtime	ActualX	ActualY	ActualHeading");
			cout.newLine();
			while (!run.Finished()) {
				run.step();
				if (run.updated()) {
					dout.write(run.getD());
					dout.newLine();
					cout.write(run.getC());
					cout.newLine();
				}
			}
			dout.close();
			cout.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
