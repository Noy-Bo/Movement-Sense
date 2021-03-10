import pathlib
import threading
from tkinter import Tk, ttk, filedialog, Label, Entry, Button, Checkbutton, BooleanVar, Canvas, W, E, StringVar
import pickle
from Calculations.Calculations import CalculateAngles
from Processing.Sync import SyncByMovementOpenpose, SyncByMovementVicon, matchTimestamps
from Readers.BagFile import BagFileSetup
from Readers.Vicon import ViconReader
from Utilities.GraphGenerator import GenerateGraph, GenerateGraphOfCorrelation


class GuiInterface(object):
    def __init__(self):
        self.title = "Image Processing Project"
        # self.path = "C:\\Users\\markf\\Downloads\\Project\\RunMe\\"
        self.path = "C:\\Users\\markf\\Downloads\\Project\\sub003\\Squat\\"
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
        self.calculations = []

    def setupInterface(self):
        # initialize windows
        self.root = Tk()
        self.root.title(self.title)
        self.root.configure(bd=10)
        # self.root.geometry("300x200")

        # Calculation checkboxes:
        Label(self.root, text="Calculations").grid(row=0, column=0, sticky=W + E, pady=5)
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
        Label(self.root, text="Orientations").grid(row=0, column=1, sticky=W + E, pady=5)
        # add front camera button
        self.frontCheckBox = BooleanVar()
        self.frontCheckBox.set(False)
        Checkbutton(self.root, text="Front", variable=self.frontCheckBox,
                    command=lambda: self.addToOrientation('Front'), cursor="hand2").grid(row=1, column=1,
                                                                                         sticky=W)
        # add side camera button
        self.sideCheckBox = BooleanVar()
        self.sideCheckBox.set(False)
        Checkbutton(self.root, text="Side", variable=self.sideCheckBox,
                    command=lambda: self.addToOrientation('Side'), cursor="hand2").grid(row=2, column=1,
                                                                                        sticky=W)
        # add back camera button
        self.backCheckBox = BooleanVar()
        self.backCheckBox.set(False)
        Checkbutton(self.root, text="Back", variable=self.backCheckBox,
                    command=lambda: self.addToOrientation('Back'), cursor="hand2").grid(row=3, column=1,
                                                                                        sticky=W)

        # add choose file button
        Button(self.root, text="Vicon Path", width=16, command=lambda: self.browseFile(), cursor="hand2",
               activebackground="Lavender").grid(row=4, column=0, sticky=W + E, padx=15, pady=10)

        # add text box
        self.textBox = StringVar()
        self.textBox.set("Welcome")
        Label(self.root, textvariable=self.textBox).grid(row=5, column=0, sticky=W + E, columnspan=4)
        # add run button
        button_run = Button(self.root, text="Run", command=lambda: self.startMainThread(), cursor="hand2",
                            activebackground="Lavender", width=10).grid(row=4, column=1, sticky=W + E, padx=15, pady=10)
        self.root.mainloop()

    def runButton(self):
        if len(self.orientations) == 0:
            self.textBox.set("Please choose camera orientation settings")
        elif len(self.calculations) == 0:
            self.textBox.set("Please choose measurements")
        else:
            self.textBox.set("Working...")
            self.Main()
            self.textBox.set("Done!")

    def startMainThread(self):
        global funcThread
        funcThread = threading.Thread(target=self.runButton)
        funcThread.start()

    def browseFile(self):
        file = filedialog.askopenfile(parent=self.root, mode='rb', title='Choose a file')
        try:
            self.path = file.name.replace('vicon.csv', '')
        except:
            self.textBox.set("Please select a valid path")

    def addToCalculations(self, param):
        if param in self.calculations:
            self.calculations.remove(param)
            return
        self.calculations.append(param)

    def addToOrientation(self, param):
        if param in self.orientations:
            self.orientations.remove(param)
            return
        self.orientations.append(param)

    def Main(self):
        openposeSkeletonsLists = []
        openposeTimestampsLists = []

        # List of vicon skeletons
        viconSkeletons = ViconReader(self.path + 'vicon.csv')
        viconSkeletons = SyncByMovementVicon(viconSkeletons)
        # # Two lists: one of Openpose skeletons, one of timestamps
        # for i in range(len(self.orientations)):
        #     # print(self.orientations[i])
        #     self.textBox.set("Working on " + str(i + 1) + '/' + str(len(self.orientations)) + " bag file")
        #     openposeSkeletons, openposeTimestamps = BagFileSetup(self.path, self.orientations[i])
        #     openposeSkeletons, openposeTimestamps = SyncByMovementOpenpose(openposeSkeletons, openposeTimestamps)
        #     openposeSkeletonsLists.append(openposeSkeletons)
        #     openposeTimestampsLists.append(openposeTimestamps)

        # pickle.dump(openposeSkeletonsLists, open(self.path + 'loadfiles\\' + "openposeSkeletonsLists", 'wb'))
        # pickle.dump(openposeTimestampsLists, open(self.path + 'loadfiles\\' + "openposeTimestampsLists", 'wb'))
        openposeSkeletonsLists = pickle.load(open(self.path + 'loadfiles\\' + "openposeSkeletonsLists", 'rb'))
        openposeTimestampsLists = pickle.load(open(self.path + 'loadfiles\\' + "openposeTimestampsLists", 'rb'))

        self.textBox.set("Calculating measurements...")
        openposeMeasurementsMat = []
        viconMeasurementsMat = []
        for i in range(len(self.calculations)):
            # openposeTimestampsLists.append(CalculateAnglesOpenpose())
            openposeMeasurements = []
            viconMeasurements = []
            for j in range(len(self.orientations)):
                openposeData, correspondingTimestamps = CalculateAngles(openposeSkeletonsLists[j], self.calculations[i],
                                                                        openposeTimestampsLists[j])
                openposeMeasurements.append([openposeData, correspondingTimestamps])
                viconData = CalculateAngles(viconSkeletons, self.calculations[i])
                cutViconData = matchTimestamps(openposeTimestampsLists[j], viconData)
                viconMeasurements.append(cutViconData)
            openposeMeasurementsMat.append(openposeMeasurements)
            viconMeasurementsMat.append(viconMeasurements)

        self.textBox.set("Exporting graphs...")
        graphsOutput = self.path + 'Graphs\\'
        pathlib.Path(graphsOutput).mkdir(parents=True, exist_ok=True)
        for i in range(len(self.calculations)):
            for j in range(len(self.orientations)):
                GenerateGraph(openposeMeasurementsMat[i][j][1], openposeMeasurementsMat[i][j][0],
                              viconMeasurementsMat[i][j], self.calculations[i], self.orientations[j], graphsOutput)
                GenerateGraphOfCorrelation(openposeMeasurementsMat[i][j][0], viconMeasurementsMat[i][j],
                                           self.calculations[i] + '_' + self.orientations[j], graphsOutput)
