package robotstats;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;
import java.util.Vector;

public class Cruncher {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String ControlFileName = args[0];
		String DataFileName = args[1];

		try {
			// Open the file that is the first
			// command line parameter
			System.out.println("Attempting to open " + DataFileName);
			FileReader fstream;
			fstream = new FileReader(DataFileName);
			// Convert our input stream to a DataInputStream
			BufferedReader in = new BufferedReader(fstream);
			System.out.println("File Open!" + DataFileName);
			System.out.println("Attempting to open " + ControlFileName);
			BufferedReader cin = new BufferedReader(new FileReader(ControlFileName));
			System.out.println("File Open! " + ControlFileName);
			// Continue to read lines while
			// there are still some left to read
			// Blow past the header
			in.readLine();
			cin.readLine();
			String line = "";
			String cline = "";
			Integer count, ecount;
			count = ecount = 0;
			Double sumX, sumY, esumX, esumY;
			sumX = sumY = esumX = esumY = 0.0;
			Double sumSquareX, sumSquareY, esumSquareX, esumSquareY;
			sumSquareX = sumSquareY = esumSquareX = esumSquareY = 0.0;

			while ((line = in.readLine()) != null && (cline = cin.readLine()) != null) {
				// pull down a line, then crush it
				// System.out.println("Reading a Line!");
				Scanner s = new Scanner(line);
				Scanner c = new Scanner(cline);
				// Skip TS
				s.next();
				// Values
				Double X = s.nextDouble();
				Double Y = s.nextDouble();
				// Resids
				Double PX = s.nextDouble();
				Double PY = s.nextDouble();
				// Control Values
				c.next();
				c.next();
				Double CX = c.nextDouble();
				Double CY = c.nextDouble();

				count++;
				ecount++;

				Double ErrX = Math.abs(CX - X);
				Double ErrY = Math.abs(CY - Y);

				sumX += PX;
				sumY += PY;
				esumX += ErrX;
				esumY += ErrY;

				sumSquareX += PX * PX;
				sumSquareY += PY * PY;
				esumSquareX += ErrX * ErrX;
				esumSquareY += ErrY * ErrY;

				if (count % 1000 == 0)
					System.out.print(".");

			}

			in.close();
			cin.close();

			Double meanX = sumX / count;
			Double meanY = sumY / count;
			Double emeanX = esumX / ecount;
			Double emeanY = esumY / ecount;

			Double stdX = Math.sqrt(sumSquareX / count - meanX * meanX);
			Double stdY = Math.sqrt(sumSquareY / count - meanY * meanY);
			Double estdX = Math.sqrt(esumSquareX / count - emeanX * emeanX);
			Double estdY = Math.sqrt(esumSquareY / count - emeanY * emeanY);

			System.out.println("Results!");
			System.out.println("Resid Mean: " + meanX + "," + meanY);
			System.out.println("Resid STD: " + stdX + "," + stdY);
			System.out.println("Error Mean: " + emeanX + "," + emeanY);
			System.out.println("Error STD: " + estdX + "," + estdY);

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
