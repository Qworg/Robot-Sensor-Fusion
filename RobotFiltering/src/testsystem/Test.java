package testsystem;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;
import java.util.Vector;

import filters.*;
import controllers.*;

public class Test {
	private Vector<Filter> FilterList = new Vector<Filter>();

	private boolean DoneCrunching = false;

	public Test(String Filename, Data DataRun) {
		if (Filename != null) {
			// Open the file that is the first
			// command line parameter
			FileReader fstream;
			try {
				fstream = new FileReader(Filename);
				// Convert our input stream to a DataInputStream
				BufferedReader in = new BufferedReader(fstream);
				System.out.println("Reading a Test file! " + Filename);
				String line;
				String filter;
				String controller;
				String option;
				Double optionnum;
				Double dQ;
				Double dR;
				Double Alpha, Beta, Kappa;
				String FuzzyFileName;
				int NumFilters;
				int NumParticles;

				while ((line = in.readLine()) != null) {
					// pull down a line, then crush it
					//System.out.println(line);
					Scanner s = new Scanner(line);
					filter = s.next();
					//System.out.println(filter);
					if (filter.contains("#")) {
						// Skipping - commented out.
						System.out.println("Skipped - Commented Out - " + line);

					} else if (filter.contains("EKF")) {
						controller = s.next();
						if (controller.contains("-T")) {
							// Okay, we're looking at a traditional EKF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									FilterList.add(new EKF(new NoControlEKF(), DataRun, dQ, dR));
									// Filter created!
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-F")) {
							// Okay, we're looking at a fuzzy EKF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										// Okay, we're correct so far
										FuzzyFileName = s.next();
										FilterList.add(new EKF(new FuzzyEKF(FuzzyFileName), DataRun, dQ, dR));
										// Filter created!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}

								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-D")) {
							System.out.println("Skipping -Ds, sorry...");

						} else if (controller.contains("-MH")) {
							// Okay, we're looking at a Multihypothesis one
							NumFilters = s.nextInt();
							Vector<Double> tempQ = new Vector<Double>();
							Vector<Double> tempR = new Vector<Double>();
							for (int i = 0; i < NumFilters; i++) {
								option = s.next();
								if (option.contains("-Q")) {
									// Okay, we're correct so far
									tempQ.add(s.nextDouble());
									option = s.next();
									if (option.contains("-R")) {
										// Okay, we're correct so far
										tempR.add(s.nextDouble());
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							}
							option = s.next();
							if (option.contains("-T")) {
								MHEKF tempMH = new MHEKF(new NoControlEKF(), DataRun, tempQ.get(0), tempR.get(0));
								for (int i = 1; i < NumFilters; i++) {
									tempMH.setControl(new NoControlEKF(), tempQ.get(i), tempR.get(i));
								}
								FilterList.add(tempMH);
							} else if (option.contains("-F")) {
								FuzzyFileName = s.next();
								MHEKF tempMH = new MHEKF(new FuzzyEKF(FuzzyFileName), DataRun, tempQ.get(0), tempR.get(0));
								for (int i = 0; i < NumFilters; i++) {
									tempMH.setControl(new FuzzyEKF(FuzzyFileName), tempQ.get(i), tempR.get(i));
								}
								FilterList.add(tempMH);
							} else {
								System.out.println("Misformed Controller!");
								break;
							}
						} else {
							System.out.println("Misformed Controller!");
							break;
						}

					} else if (filter.contains("SPKF")) {
						//System.out.println("In SPKF!");
						controller = s.next();
						if (controller.contains("-T")) {
							// Okay, we're looking at a traditional SPKF
							//System.out.println("Traditional SPKF");
							option = s.next();
							//System.out.println(option);
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								//System.out.println("Got Q " + dQ);
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									//System.out.println("Got R " + dR);
									// System.out.println(DataRun.GetGPSEntry(40).toString());
									FilterList.add(new SPKF(new NoControlSPKF(9), DataRun, dQ, dR));
									//System.out.println("Filter Built!");
									// Filter created!
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-TO")) {
							// Okay, we're looking at a traditional SPKF w/
							// options
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									Alpha = s.nextDouble();
									Beta = s.nextDouble();
									Kappa = s.nextDouble();
									FilterList.add(new SPKF(new NoControlSPKF(9), DataRun, dQ, dR, Alpha, Beta, Kappa));
									// Filter created!
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-F")) {
							// Okay, we're looking at a fuzzy SPKF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										// Okay, we're correct so far
										FuzzyFileName = s.next();
										FilterList.add(new SPKF(new FuzzySPKF(FuzzyFileName, 9), DataRun, dQ, dR));
										// Filter created!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}

								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-FO")) {
							// Okay, we're looking at a fuzzy SPKF with Options!
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										// Okay, we're correct so far
										FuzzyFileName = s.next();
										Alpha = s.nextDouble();
										Beta = s.nextDouble();
										Kappa = s.nextDouble();
										FilterList.add(new SPKF(new FuzzySPKF(FuzzyFileName, 9), DataRun, dQ, dR, Alpha, Beta, Kappa));
										// Filter created!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}

								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-DF")) {
							// Okay, we're looking at a double fuzzy SPKF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										// Okay, we're correct so far
										FuzzyFileName = s.next();
										FilterList.add(new SPKF(new FuzzySPKF(FuzzyFileName, "dfkfis.txt", 9), DataRun, dQ, dR));
										// Filter created!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}

								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-DFO")) {
							// Okay, we're looking at a double fuzzy SPKF with
							// Options!
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										// Okay, we're correct so far
										FuzzyFileName = s.next();
										Alpha = s.nextDouble();
										Beta = s.nextDouble();
										Kappa = s.nextDouble();
										FilterList.add(new SPKF(new FuzzySPKF(FuzzyFileName, "dfkfis.txt", 9), DataRun, dQ, dR, Alpha, Beta, Kappa));
										// Filter created!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}

								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-D")) {
							System.out.println("Skipping -Ds, sorry...");

						} else if (controller.contains("-MH")) {
							// Okay, we're looking at a Multihypothesis one
							NumFilters = s.nextInt();
							Vector<Double> tempQ = new Vector<Double>();
							Vector<Double> tempR = new Vector<Double>();
							for (int i = 0; i < NumFilters; i++) {
								option = s.next();
								if (option.contains("-Q")) {
									// Okay, we're correct so far
									tempQ.add(s.nextDouble());
									option = s.next();
									if (option.contains("-R")) {
										// Okay, we're correct so far
										tempR.add(s.nextDouble());
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							}
							option = s.next();
							if (option.contains("-T")) {
								MHSPKF tempMH = new MHSPKF(new NoControlSPKF(9), DataRun, tempQ.get(0), tempR.get(0));
								for (int i = 1; i < NumFilters; i++) {
									tempMH.setControl(new NoControlSPKF(9), tempQ.get(i), tempR.get(i));
								}
								FilterList.add(tempMH);
							} else if (option.contains("-F")) {
								FuzzyFileName = s.next();
								MHSPKF tempMH = new MHSPKF(new FuzzySPKF(FuzzyFileName, 9), DataRun, tempQ.get(0), tempR.get(0));
								for (int i = 0; i < NumFilters; i++) {
									tempMH.setControl(new FuzzySPKF(FuzzyFileName, 9), tempQ.get(i), tempR.get(i));
								}
								FilterList.add(tempMH);
							} else {
								System.out.println("Misformed Controller!");
								break;
							}
						} else {
							System.out.println("Misformed Controller!");
							break;
						}
					} else if (filter.contains("PF")) {
						controller = s.next();
						if (controller.contains("-ET")) {
							// This is a trad EKF PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-P")) {
										// Okay, we're correct so far
										NumParticles = s.nextInt();
										FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new NoControlEKF(), 2, dQ, dR));
										// Filter Added!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-ST")) {
							// This is a trad SPKF PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-P")) {
										// Okay, we're correct so far
										NumParticles = s.nextInt();
										FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new NoControlSPKF(9), 3, dQ, dR));
										// Filter Added!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-EF")) {
							// This is a Fuzzy EKF PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										FuzzyFileName = s.next();
										option = s.next();
										if (option.contains("-P")) {
											// Okay, we're correct so far
											NumParticles = s.nextInt();
											FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new FuzzyEKF(FuzzyFileName), 2, dQ, dR));
											// Filter Added!
										} else {
											System.out.println("Misformed Controller!");
											break;
										}
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-SF")) {
							// This is a Fuzzy SPKF PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										FuzzyFileName = s.next();
										option = s.next();
										if (option.contains("-P")) {
											// Okay, we're correct so far
											NumParticles = s.nextInt();
											FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new FuzzySPKF(FuzzyFileName, 9), 3, dQ, dR));
											// Filter Added!
										} else {
											System.out.println("Misformed Controller!");
											break;
										}
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-SFO")) {
							// This is a Fuzzy SPKF PF with Options
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										FuzzyFileName = s.next();
										option = s.next();
										if (option.contains("-P")) {
											// Okay, we're correct so far
											NumParticles = s.nextInt();
											Alpha = s.nextDouble();
											Beta = s.nextDouble();
											Kappa = s.nextDouble();
											FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new FuzzySPKF(FuzzyFileName, 9), 3, dQ, dR, Alpha, Beta, Kappa));
											// Filter Added!
										} else {
											System.out.println("Misformed Controller!");
											break;
										}
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-SDF")) {
							// This is a Double Fuzzy SPKF PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										FuzzyFileName = s.next();
										option = s.next();
										if (option.contains("-P")) {
											// Okay, we're correct so far
											NumParticles = s.nextInt();
											FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new FuzzySPKF(FuzzyFileName, "dfkfis.txt", 9), 3, dQ, dR));
											// Filter Added!
										} else {
											System.out.println("Misformed Controller!");
											break;
										}
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-SDFO")) {
							// This is a Double Fuzzy SPKF PF with options
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-I")) {
										FuzzyFileName = s.next();
										option = s.next();
										if (option.contains("-P")) {
											// Okay, we're correct so far
											NumParticles = s.nextInt();
											Alpha = s.nextDouble();
											Beta = s.nextDouble();
											Kappa = s.nextDouble();
											FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new FuzzySPKF(FuzzyFileName, "dfkfis.txt", 9), 3, dQ, dR, Alpha, Beta, Kappa));
											// Filter Added!
										} else {
											System.out.println("Misformed Controller!");
											break;
										}
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-STO")) {
							// This is a trad SPKF PF with options
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-P")) {
										// Okay, we're correct so far
										NumParticles = s.nextInt();
										Alpha = s.nextDouble();
										Beta = s.nextDouble();
										Kappa = s.nextDouble();
										FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new NoControlSPKF(9), 3, dQ, dR, Alpha, Beta, Kappa));
										// Filter Added!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else if (controller.contains("-ED")) {
							System.out.println("Skipping -Ds, sorry...");

						} else if (controller.contains("-SD")) {
							System.out.println("Skipping -Ds, sorry...");

						} else if (controller.contains("-T")) {
							// This is a traditional PF
							option = s.next();
							if (option.contains("-Q")) {
								// Okay, we're correct so far
								optionnum = s.nextDouble();
								dQ = optionnum;
								option = s.next();
								if (option.contains("-R")) {
									// Okay, we're correct so far
									dR = s.nextDouble();
									option = s.next();
									if (option.contains("-P")) {
										// Okay, we're correct so far
										NumParticles = s.nextInt();
										FilterList.add(new PF(new NoControlPF(), DataRun, NumParticles, new NoControlSPKF(9), 1, dQ, dR));
										// Filter Added!
									} else {
										System.out.println("Misformed Controller!");
										break;
									}
								} else {
									System.out.println("Misformed Controller!");
									break;
								}
							} else {
								System.out.println("Misformed Controller!");
								break;
							}

						} else {
							System.out.println("Misformed Controller!");
							break;
						}

					} else {
						System.out.println("Wrong Filter Descript!");
						break;
					}

				}

			} catch (FileNotFoundException e) {
				System.out.println("Couldn't find Test File!");
				e.printStackTrace();
			} catch (IOException e) {
				System.out.println("Oh man, test file explosion!");
				e.printStackTrace();
			}

		} else {
			System.out.println("File Name is bad for the Test File");
		}
		DoneCrunching = true;

	}

	public Vector<Filter> GetFilterList() {
		if (DoneCrunching) {
			// Okay, take it.
			return FilterList;
		} else
			System.out.println("Not Done Crunching!");
		return null;
	}

	public boolean FinishedParsing() {
		return DoneCrunching;
	}

}
