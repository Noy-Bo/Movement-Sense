from tkinter import Tk, ttk, filedialog, Label, Entry, Button, Checkbutton, BooleanVar, Canvas, W, E, StringVar

from Calculations.Calculations import CalculateAngles, CalculateMeasurement
from Processing.Sync import SyncByMovementOpenpose, SyncByMovementVicon, matchTimestamps
from Readers.BagFile import BagFileSetup
from Readers.Vicon import ViconReader
from Utilities.GraphGenerator import GenerateGraph


class GuiInterface(object):
    def __init__(self):
        self.title = "Image Processing Project"
        # self.path = "C:\\Users\\markf\\Downloads\\Project\\RunMe\\"
        self.path = "C:\\Age_Estimation_Project\\bag_files\sub003\\Squat\\"
        self.root = None
        self.combo = None
        self.textBox = None

        self.kneesCheckBox = None
        self.massCheckBox = None
        self.KyphosisCheckBox = None
        self.frontCheckBox = None
        self.sideCheckBox = None
        self.backCheckBox = None
        self.orientations = []
        self.calculation = []

    def setupInterface(self):
        # initialize windows
        self.root = Tk()
        self.root.title(self.title)
        self.root.configure(bd=10)
        # self.root.geometry("300x200")

        # # add label and choises of algorithms
        # Label(self.root, text="Algorithm:").grid(row=0, column=0, sticky=W)
        # self.combo = ttk.Combobox(self.root, width=10, values=self.algorithms)
        # self.combo.current(4)
        # self.combo.grid(row=0, column=1, sticky=W)

        # # add label and time limit entry
        # Label(self.root, text="Time Limit").grid(row=1, column=0, sticky=W)
        # self.runtimeEntry = Entry(self.root, width=13)
        # self.runtimeEntry.grid(row=1, column=1, sticky=W)
        # self.runtimeEntry.insert(0, "100")

        # Calculation checkboxes:
        Label(self.root, text="Calculations").grid(row=0, column=0, sticky=W, pady=5)
        # add kyphosis button
        self.KyphosisCheckBox = BooleanVar()
        self.KyphosisCheckBox.set(False)
        Checkbutton(self.root, text="Kyphosis", variable=self.KyphosisCheckBox,
                    command=lambda: self.addToCalculations('Kyphosis'), cursor="hand2").grid(row=1, column=0,
                                                                                             sticky=W)
        # add mass center button
        self.massCheckBox = BooleanVar()
        self.massCheckBox.set(False)
        Checkbutton(self.root, text="Mass Center", variable=self.massCheckBox,
                    command=lambda: self.addToCalculations('Mass'), cursor="hand2").grid(row=2, column=0,
                                                                                         sticky=W)
        # add knees button
        self.kneesCheckBox = BooleanVar()
        self.kneesCheckBox.set(False)
        Checkbutton(self.root, text="Knees", variable=self.kneesCheckBox,
                    command=lambda: self.addToCalculations('Knees'), cursor="hand2").grid(row=3, column=0,
                                                                                          sticky=W)

        # Orientation checkboxes:
        Label(self.root, text="Orientations").grid(row=0, column=1, sticky=E, pady=5)
        # add front button
        self.frontCheckBox = BooleanVar()
        self.frontCheckBox.set(False)
        Checkbutton(self.root, text="Front", variable=self.frontCheckBox,
                    command=lambda: self.addToOrientation('Front'), cursor="hand2").grid(row=1, column=1,
                                                                                         sticky=W)
        # add side center button
        self.sideCheckBox = BooleanVar()
        self.sideCheckBox.set(False)
        Checkbutton(self.root, text="Side", variable=self.sideCheckBox,
                    command=lambda: self.addToOrientation('Side'), cursor="hand2").grid(row=2, column=1,
                                                                                        sticky=W)
        # add back button
        self.backCheckBox = BooleanVar()
        self.backCheckBox.set(False)
        Checkbutton(self.root, text="Back", variable=self.backCheckBox,
                    command=lambda: self.addToOrientation('Back'), cursor="hand2").grid(row=3, column=1,
                                                                                        sticky=W)

        # add choose file button
        Button(self.root, text="Vicon Path", width=16, command=lambda: self.browseFile(), cursor="hand2",
               activebackground="Lavender").grid(row=4, column=0, sticky=W + E, padx=10, pady=10)

        # add run button
        Label(self.root, textvariable=self.textBox).grid(row=1, column=2, sticky=W + E, columnspan=4)
        button_run = Button(self.root, text="Run", command=lambda: self.runButton(), cursor="hand2",
                            activebackground="Lavender").grid(row=4, column=1, sticky=W + E, padx=10, pady=10)

        self.textBox = StringVar()
        self.textBox.set("Welcome")

        self.root.mainloop()

    def runButton(self):
        self.Main(self.path, self.orientations, self.calculation)
        # self.mazeScreen.clear()
        # maxRunTime = int(self.runtimeEntry.get())
        # algorithmName, startNode, goalNode, mazeSize, maze = Utilities.readInstance(self.path)
        # algorithm, isHeuristic = self.getAlgorithmFromString(self.combo.get(), self.visualCheckBox.get())
        self.textBox.set(", please wait...")
        # self.mazeScreen.update()
        # if isHeuristic is True:
        #     algorithm(maze, maxRunTime, "minimumMoves")
        #     # algorithm(maze, maxRunTime, "movesCount")
        # else:
        #     algorithm(maze, maxRunTime)
        self.textBox.set("Finished running. See OutputResult.txt for results")
        # self.mazeScreen.update()

    def browseFile(self):
        file = filedialog.askopenfile(parent=self.root, mode='rb', title='Choose a file')
        # algorithmName, startNode, goalNode, mazeSize, maze = Utilities.readInstance(file.name)
        # for item in self.algorithms:
        #     if algorithmName.lower() == item.lower():
        #         self.combo.current(self.algorithms.index(item))
        self.path = file.name

    def addToCalculations(self, param):
        if param in self.calculation:
            self.calculation.remove(param)
            return
        self.calculation.append(param)

    def addToOrientation(self, param):
        if param in self.orientations:
            self.orientations.remove(param)
            return
        self.orientations.append(param)

    def Main(self, path, orientations, calculations):
        openposeSkeletonsLists = []
        openposeTimestampsLists = []

        # List of vicon skeletons
        viconSkeletons = ViconReader(path + 'vicon.csv')
        viconSkeletons = SyncByMovementVicon(viconSkeletons)
        # Two lists: one of Openpose skeletons, one of timestamps
        for i in range(len(orientations)):
            print(orientations[i])
            openposeSkeletons, openposeTimestamps = BagFileSetup(path, orientations[i])
            openposeSkeletons, openposeTimestamps = SyncByMovementOpenpose(openposeSkeletons, openposeTimestamps)
            openposeSkeletonsLists.append(openposeSkeletons)
            openposeTimestampsLists.append(openposeTimestamps)

        openposeMeasurementsMat = []
        viconMeasurementsMat = []
        for i in range(len(calculations)):
            # openposeTimestampsLists.append(CalculateAnglesOpenpose())
            openposeMeasurements = []
            viconMeasurements = []
            for j in range(len(orientations)):
                openposeData, correspondingTimestamps = CalculateMeasurement(openposeSkeletonsLists[j], calculations[i], openposeTimestampsLists[j])
                openposeMeasurements.append([openposeData, correspondingTimestamps])
                viconData = CalculateMeasurement(viconSkeletons, calculations[i])
                cutViconData = matchTimestamps(openposeTimestampsLists[j], viconData)
                viconMeasurements.append(cutViconData)

            openposeMeasurementsMat.append(openposeMeasurements)
            viconMeasurementsMat.append(viconMeasurements)

        for i in range(len(calculations)):
            for j in range(len(orientations)):
                print('Graph of ' + calculations[i] + ' from ' + orientations[j] + ' Camera')
                GenerateGraph(openposeMeasurementsMat[i][j][0], openposeMeasurementsMat[i][j][1], viconMeasurementsMat[i][j], calculations[i], orientations[j],"C:\Age_Estimation_Project\\bag_files")