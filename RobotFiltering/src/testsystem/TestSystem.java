package testsystem;

import java.util.Date;
import java.util.Vector;

import filters.Filter;

import testsystem.Data;

public class TestSystem {

	/**
	 * @param args
	 */

	public static void main(String[] args) {
		String DataFileName = args[0];
		String TestFileName = args[1];

		// Load the DataFile and Parse
		System.out.println("Loading Data File");
		Data DataRun = new Data(DataFileName);	
		System.out.println("Finished Parsing Data File");
		// Load the TestFile and Parse
		System.out.println("Loading Test File");
		Test TestList = new Test(TestFileName, DataRun);

		while (!TestList.FinishedParsing()) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Finished Parsing Test File");
		
		Vector<Filter> Filters = TestList.GetFilterList();
		Date CurDate = new Date();
		String DateString = CurDate.toString();
		for (int i = 0; i < Filters.size(); i++) {
			Filter tempFilter = Filters.get(i);
			tempFilter.startProcess();

			while (!tempFilter.finishedProcess()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			
			if (i<10)
				tempFilter.outputData("0"+i+"-Filter-");
			else
				tempFilter.outputData(i+"-Filter-");
		}
		
		System.out.println("Done!");
	}

}
