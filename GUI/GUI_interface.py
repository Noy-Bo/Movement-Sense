import glob
import os
import pathlib
import pickle
import sys
import threading
from tkinter import Tk, ttk, filedialog, Label, Entry, Button, Checkbutton, BooleanVar, Canvas, W, E, StringVar, Menu, \
    Toplevel, messagebox

from Calculations.Calculations import CalculateAngles, CalculateMeasurement
from Processing.LogGenerator import GenerateLog
from Processing.Sync import SyncByMovementOpenpose, SyncByMovementVicon, matchTimestamps, removeOutliers
from Readers.BagFile import BagFileSetup
from Readers.Vicon import ViconReader
from Utilities.Excel import GenerateExcel
from Utilities.GraphGenerator import GenerateGraph, GenerateGraphOfCorrelation

INSTRUCTIONS = "First, organize the folder according to the next diagram:\n\n" \
               "path/\n" \
               "   vicon.csv\n" \
               "   front.bag\n" \
               "   front_log.json\n" \
               "   side.bag\n" \
               "   side_log.json\n" \
               "   back.bag\n" \
               "   back_log.json\n" \
               "   front_openpose/\n" \
               "       1.607859213e+12_keypoints.json\n" \
               "       ...\n" \
               "   side_openpose/\n" \
               "       1.607859213e+12_keypoints.json\n" \
               "       ...\n" \
               "   back_openpose/\n" \
               "       1.607859213e+12_keypoints.json\n" \
               "       ...\n\n" \
               "-   All rotations are " \
               "optional\n-   xxx_log.json can be generated using this tool\n-   xxx_openpose is generated using the " \
               "Openpose tool (separate)\n\nThen, click ‘Vicon Path’ to navigate the program to your vicon.csv.\nIf " \
               "xxx_log.json are missing, please select the requested orientations and each camera’s angle, " \
               "and press ‘Generate logs’.\nThen, to get measurements graphs, please choose the requested " \
               "calculations and camera orientations used and press ‘Run’.\nThe program will output the requested " \
               "graphs to the same path originally used, inside a new folder named ‘Graphs’.\n\n" \
               "* Press 'Reset' to remove all chached files"


class GuiInterface(object):
    def __init__(self):
        self.title = "Movement Sense"
        # self.path = "C:\\Age_Estimation_Project\\bag_files\\sub005\\left\\"
        self.path = 'C:/Users/markf/Downloads/Project/sub003/Squat/'
        # self.path = "C:\\Age_Estimation_Project\\bag_files\sub003\\Squat\\"
        self.root = None
        self.combo = None
        self.instructionsWindow = None
        self.aboutWindow = None
        self.textBox = None
        self.buttonRun = None
        self.buttonLog = None
        self.buttonBrowse = None

        self.checkButtonKnees = None
        self.checkButtonMass = None
        self.checkButtonKyphosis = None
        self.checkButtonFront = None
        self.checkButtonSide = None
        self.checkButtonBack = None
        self.kneesCheckBox = None
        self.massCheckBox = None
        self.KyphosisCheckBox = None
        self.frontCheckBox = None
        self.sideCheckBox = None
        self.backCheckBox = None
        self.orientations = []
        self.calculations = []
        self.cameraOrientations = ['0', '90 right', '180', '90 left']
        self.frontOrientation = None
        self.sideOrientation = None
        self.backOrientation = None

    def setupInterface(self):
        # initialize windows
        self.root = Tk()
        self.root.title(self.title)
        self.root.configure(bd=10)
        # self.root.geometry("300x200")

        # Column 0
        # Calculation checkboxes:
        Label(self.root, text="Calculation").grid(row=0, column=0, sticky=W, pady=5)
        # add kyphosis button
        self.KyphosisCheckBox = BooleanVar()
        self.KyphosisCheckBox.set(False)
        self.checkButtonKyphosis = Checkbutton(self.root, text="Kyphosis", variable=self.KyphosisCheckBox,
                    command=lambda: self.addToCalculations('Kyphosis'), cursor="hand2")
        self.checkButtonKyphosis.grid(row=1, column=0, sticky=W)
        # add mass center button
        self.massCheckBox = BooleanVar()
        self.massCheckBox.set(False)
        self.checkButtonMass = Checkbutton(self.root, text="Mass Center", variable=self.massCheckBox,
                    command=lambda: self.addToCalculations('Mass'), cursor="hand2")
        self.checkButtonMass.grid(row=2, column=0, sticky=W)
        # add knees button
        self.kneesCheckBox = BooleanVar()
        self.kneesCheckBox.set(False)
        self.checkButtonKnees = Checkbutton(self.root, text="Knees", variable=self.kneesCheckBox,
                    command=lambda: self.addToCalculations('Knees'), cursor="hand2")
        self.checkButtonKnees.grid(row=3, column=0, sticky=W)
        # Column 1
        # separator line
        separator = ttk.Separator(self.root, orient='vertical')
        separator.grid(row=0, column=1, rowspan=4, sticky="ns", padx=5)

        # Column 2
        # Orientation checkboxes:
        Label(self.root, text="Orientation").grid(row=0, column=2, sticky=W + E, pady=5)

        # add front camera button
        self.frontCheckBox = BooleanVar()
        self.frontCheckBox.set(False)
        self.checkButtonFront = Checkbutton(self.root, text="Front", variable=self.frontCheckBox,
                    command=lambda: self.addToOrientation('Front'), cursor="hand2")
        self.checkButtonFront.grid(row=1, column=2, sticky=W)

        # add side camera button
        self.sideCheckBox = BooleanVar()
        self.sideCheckBox.set(False)
        self.checkButtonSide = Checkbutton(self.root, text="Side", variable=self.sideCheckBox,
                    command=lambda: self.addToOrientation('Side'), cursor="hand2")
        self.checkButtonSide.grid(row=2, column=2, sticky=W)
        # add back camera button
        self.backCheckBox = BooleanVar()
        self.backCheckBox.set(False)
        self.checkButtonBack = Checkbutton(self.root, text="Back", variable=self.backCheckBox,
                    command=lambda: self.addToOrientation('Back'), cursor="hand2")
        self.checkButtonBack.grid(row=3, column=2, sticky=W)

        # Column 3
        Label(self.root, text="Camera Angle").grid(row=0, column=3, sticky=W + E, pady=5, padx=10)

        self.frontOrientation = ttk.Combobox(self.root, width=5, values=self.cameraOrientations)
        self.frontOrientation.current(1)
        self.frontOrientation.grid(row=1, column=3, sticky=W, padx=10)

        self.sideOrientation = ttk.Combobox(self.root, width=5, values=self.cameraOrientations)
        self.sideOrientation.current(1)
        self.sideOrientation.grid(row=2, column=3, sticky=W, padx=10)

        self.backOrientation = ttk.Combobox(self.root, width=5, values=self.cameraOrientations)
        self.backOrientation.current(1)
        self.backOrientation.grid(row=3, column=3, sticky=W, padx=10)

        # add choose file button
        self.buttonBrowse = Button(self.root, text="Vicon Path", width=12, command=lambda: self.browseFile(), cursor="hand2",
               activebackground="Lavender")
        self.buttonBrowse.grid(row=4, column=0, sticky=W + E, pady=10)

        # add generate log button
        self.buttonLog = Button(self.root, text="Generate logs", width=12,
                                command=lambda: self.startMainThread(self.GenerateLogs), cursor="hand2",
                                activebackground="Lavender")
        self.buttonLog.grid(row=4, column=2, sticky=W + E, pady=10)

        # add run button
        self.buttonRun = Button(self.root, text="Run", command=lambda: self.startMainThread(self.runButton),
                                cursor="hand2",
                                activebackground="Lavender", width=10)
        self.buttonRun.grid(row=4, column=3, pady=10)

        # add toolbar
        menubar = Menu(self.root, tearoff=0)
        self.root.config(menu=menubar)
        menubar.add_command(label='Instructions', command=lambda: self.openInstructionsWindow())
        menubar.add_command(label='Reset', command=lambda: self.resetPickle())
        menubar.add_command(label='About', command=lambda: self.openAboutWindow())


        # add text box
        self.textBox = StringVar()
        self.textBox.set("Welcome")
        Label(self.root, textvariable=self.textBox).grid(row=5, column=0, sticky=W + E, columnspan=4)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.root.destroy()

    def resetPickle(self):
        path = self.path + 'loadfiles/'
        if os.path.isdir(path):
            files = glob.glob(path + '*')
            if len(files) > 0:
                if messagebox.askokcancel("Yes", "Delete all cached files?"):
                    for f in files:
                        os.remove(f)
                    self.textBox.set("Cache cleared")
            else:
                self.textBox.set("Cache is already clear")
        else:
            self.textBox.set("No cache folder in here")

    def openInstructionsWindow(self):
        if self.instructionsWindow is None:
            self.instructionsWindow = Toplevel(self.root)
            self.instructionsWindow.title("Instructions")
            self.instructionsWindow.geometry("710x460")
            Label(self.instructionsWindow, text=INSTRUCTIONS, justify='left').pack()
            self.instructionsWindow.protocol("WM_DELETE_WINDOW", self.onInstructionsClose)

    def openAboutWindow(self):
        if self.aboutWindow is None:
            self.aboutWindow = Toplevel(self.root)
            self.aboutWindow.title("About")
            self.aboutWindow.geometry("50x100")
            Label(self.aboutWindow, text="Noy Boutbul\nMark Fesenko").pack()
            self.aboutWindow.protocol("WM_DELETE_WINDOW", self.onAboutClose)

    def onInstructionsClose(self):
        self.instructionsWindow.destroy()
        self.instructionsWindow = None

    def onAboutClose(self):
        self.aboutWindow.destroy()
        self.aboutWindow = None

    def runButton(self):
        if len(self.orientations) == 0:
            self.textBox.set("Please choose camera orientation settings")
        elif len(self.calculations) == 0:
            self.textBox.set("Please choose measurements")
        else:
            self.textBox.set("Working...")
            self.disableButtons()
            self.Main()
            self.textBox.set("Done!")
            self.enableButtons()

    def disableButtons(self):
        self.buttonRun.configure(state='disabled')  # must be active, disabled, or normal
        self.buttonLog.configure(state='disabled')
        self.buttonBrowse.configure(state='disabled')
        self.checkButtonFront.config(state='disabled')
        self.checkButtonSide.config(state='disabled')
        self.checkButtonBack.config(state='disabled')
        self.checkButtonKyphosis.config(state='disabled')
        self.checkButtonKnees.config(state='disabled')
        self.checkButtonMass.config(state='disabled')
        self.frontOrientation.config(state='disabled')
        self.sideOrientation.config(state='disabled')
        self.backOrientation.config(state='disabled')

    def enableButtons(self):
        self.buttonRun.configure(state='normal')  # must be active, disabled, or normal
        self.buttonLog.configure(state='normal')
        self.buttonBrowse.configure(state='normal')
        self.checkButtonFront.config(state='normal')
        self.checkButtonSide.config(state='normal')
        self.checkButtonBack.config(state='normal')
        self.checkButtonKyphosis.config(state='normal')
        self.checkButtonKnees.config(state='normal')
        self.checkButtonMass.config(state='normal')
        self.frontOrientation.config(state='normal')
        self.sideOrientation.config(state='normal')
        self.backOrientation.config(state='normal')

    def startMainThread(self, button):
        global funcThread
        funcThread = threading.Thread(target=button)
        funcThread.start()

    def browseFile(self):
        file = filedialog.askopenfile(parent=self.root, mode='rb', title='Choose a file')
        try:
            self.path = file.name.replace('vicon.csv', '')
        except:
            self.textBox.set("Please select a valid path")

    def translateAngle(self, camera):
        angle = self.cameraOrientations.index(camera.get())
        return 360 - (90 * angle)

    def addToCalculations(self, param):
        if param in self.calculations:
            self.calculations.remove(param)
            return
        self.calculations.append(param)
        self.calculations = sorted(self.calculations)

    def addToOrientation(self, param):
        if param in self.orientations:
            self.orientations.remove(param)
            return
        self.orientations.append(param)
        self.orientations = sorted(self.orientations)

    def getRotationAngle(self, orientation):
        name = orientation.lower() + 'Orientation'
        attr = getattr(self, name)
        rotationAngle = self.translateAngle(attr)
        return rotationAngle

    def GenerateLogs(self):
        if len(self.orientations) == 0:
            self.textBox.set("Please choose orientations")
            return
        else:
            self.disableButtons()
        for i in range(len(self.orientations)):
            self.textBox.set("Generating " + self.orientations[i] + " orientation log files...")
            rotationAngle = self.getRotationAngle(self.orientations[i])
            try:
                GenerateLog(self.path, self.orientations[i], rotationAngle)
            except:
                self.textBox.set("Path error. Please choose correct path")
                self.disableButtons()
                return
        if len(self.orientations) > 0:
            self.textBox.set("Done generating log files!")
            self.enableButtons()

    def Main(self):
        openposeSkeletonsLists = []
        openposeTimestampsLists = []

        # List of vicon skeletons
        viconSkeletons = ViconReader(self.path + 'vicon.csv')
        viconSkeletons = SyncByMovementVicon(viconSkeletons)

        picklePath = self.path + 'loadfiles\\'
        pathlib.Path(picklePath).mkdir(parents=True, exist_ok=True)
        if os.path.exists(self.path + 'loadfiles'):
            angles = [self.translateAngle(self.frontOrientation), self.translateAngle(self.sideOrientation), self.translateAngle(self.backOrientation)]
            pickleOrientations = pickle.load(open(self.path + 'loadfiles\\' + "orientations", 'rb'))
            pickleAngles = pickle.load(open(self.path + 'loadfiles\\' + "angles", 'rb'))
            openposeSkeletonsLists = pickle.load(open(self.path + 'loadfiles\\' + "openposeSkeletonsLists", 'rb'))
            openposeTimestampsLists = pickle.load(open(self.path + 'loadfiles\\' + "openposeTimestampsLists", 'rb'))
            if self.orientations != pickleOrientations or angles != pickleAngles:
                # Two lists: one of Openpose skeletons, one of timestamps
                for i in range(len(self.orientations)):
                    self.textBox.set("Working on " + str(i + 1) + '/' + str(len(self.orientations)) + " bag file")
                    rotationAngle = self.getRotationAngle(self.orientations[i])
                    openposeSkeletons, openposeTimestamps = BagFileSetup(self.path, self.orientations[i], rotationAngle)
                    openposeSkeletons, openposeTimestamps = SyncByMovementOpenpose(openposeSkeletons, openposeTimestamps)
                    openposeSkeletonsLists.append(openposeSkeletons)
                    openposeTimestampsLists.append(openposeTimestamps)

                dirPath = self.path + 'loadfiles'
                try:
                    os.mkdir(dirPath)
                except OSError:
                    pass
                pickle.dump(angles, open(self.path + 'loadfiles\\' + "angles", 'wb'))
                pickle.dump(self.orientations, open(self.path + 'loadfiles\\' + "orientations", 'wb'))
                pickle.dump(openposeSkeletonsLists, open(self.path + 'loadfiles\\' + "openposeSkeletonsLists", 'wb'))
                pickle.dump(openposeTimestampsLists, open(self.path + 'loadfiles\\' + "openposeTimestampsLists", 'wb'))

        self.textBox.set("Calculating measurements...")
        openposeMeasurementsMat = []
        viconMeasurementsMat = []
        for i in range(len(self.calculations)):
            openposeMeasurements = []
            viconMeasurements = []
            for j in range(len(self.orientations)):
                openposeData, correspondingTimestamps = CalculateMeasurement(openposeSkeletonsLists[j],
                                                                             self.calculations[i],
                                                                             openposeTimestampsLists[j])
                viconData = CalculateMeasurement(viconSkeletons, self.calculations[i])
                cutViconData = matchTimestamps(openposeTimestampsLists[j], viconData)
                cutViconData, openposeData, correspondingTimestamps = removeOutliers(cutViconData, openposeData,
                                                                                     correspondingTimestamps)
                viconMeasurements.append(cutViconData)
                openposeMeasurements.append([openposeData, correspondingTimestamps])
            openposeMeasurementsMat.append(openposeMeasurements)
            viconMeasurementsMat.append(viconMeasurements)

        self.textBox.set("Exporting graphs...")
        graphsOutput = self.path + 'Graphs\\'
        excelOutput = self.path + 'Excel\\'
        pathlib.Path(graphsOutput).mkdir(parents=True, exist_ok=True)
        pathlib.Path(excelOutput).mkdir(parents=True, exist_ok=True)
        for i in range(len(self.calculations)):
            for j in range(len(self.orientations)):
                GenerateExcel(openposeMeasurementsMat[i][j][1], openposeMeasurementsMat[i][j][0],
                              viconMeasurementsMat[i][j], excelOutput,
                              self.calculations[i] + '_' + self.orientations[j])
                GenerateGraph(openposeMeasurementsMat[i][j][1], openposeMeasurementsMat[i][j][0],
                              viconMeasurementsMat[i][j], self.calculations[i], self.orientations[j], graphsOutput)
                GenerateGraphOfCorrelation(openposeMeasurementsMat[i][j][0], viconMeasurementsMat[i][j],
                                           self.calculations[i] + '_' + self.orientations[j], graphsOutput)
