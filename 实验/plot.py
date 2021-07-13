import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
column_names = ['index','curentX','curentY', 'curentYaw','TheoryX','TheoryY','LatError']


class PlotPath:

    def __init__(self, file_path):
        data = pd.read_csv(file_path,header = None, names = column_names)
        self.index =  data['index']
        self.curentX = data['curentX']
        self.curentY = data['curentY']
        self.curentYaw = data['curentYaw']
        self.TheoryX = data['TheoryX']
        self.TheoryY = data['TheoryY']
        self.LatError = data['LatError']

    def plotPath(self):
        plt.figure()
        plt.title("AGV Path (m)")
        plt.plot(self.curentX , self.curentY, label="running pose")
        plt.plot(self.TheoryX , self.TheoryY, label = "theory pose")
        
        plt.axis('equal')
        plt.legend()
        plt.grid()
        # plt.show()
    def plotError(self):
        plt.figure()
        plt.title("dis Error (m)")
        dx = self.curentX  - self.TheoryX
        dy = self.curentY - self.TheoryY
        dis = []
        for i in range(len(dx)):
            dis.append(math.sqrt(math.pow(dx[i], 2) + math.pow(dy[i], 2)))
        plt.plot(self.index ,dis)
        plt.grid()
        
    def plotLatError(self):
        plt.figure()
        plt.title("lat Error (m)")
        plt.grid()
        plt.plot(self.index ,self.LatError)
    
if __name__ == '__main__':
    plt.figure()
    myPlot = PlotPath("0702_2.txt")
    # myPlot1 = PlotPath("0702_1.txt")
    # myPlot2 = PlotPath("from12_10_2m.txt")
    myPlot.plotPath()
    # myPlot2.plotPath()
    # myPlot2.plotPath()

    myPlot.plotError()
    myPlot.plotLatError()
    # myPlot2.plotError()
    # myPlot1.plotPath()
    # myPlot1.plotError()
    # myPlot1.plotLatError()
    plt.show()



        