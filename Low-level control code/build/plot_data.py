NUMBER_OF_VARIABLES = int(11)
#TITLE_OF_THE_DIAGRAM = "KP=0.001, KI=0.001, KD=0.001, f=500Hz, Unloaded"

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import csv

#plt.style.use('_mpl-gallery')
logData = np.empty((0,NUMBER_OF_VARIABLES), str)
temporary = np.empty((1,NUMBER_OF_VARIABLES), str)
info = np.empty((0,NUMBER_OF_VARIABLES), str)
labels = np.empty((0,NUMBER_OF_VARIABLES), str)
#print(logData)
# open the file
with open('/home/laikago/catkin_humanoid/UnitreeMotorSDK_A1Go1_Sup220423/build/logData.csv' , 'r') as csvfile:
    # create the object of csv.reader()
    csv_file_reader = csv.reader(csvfile,delimiter=',')
    for row in csv_file_reader:
        temporary = [row]
        #print(row)
        #print(temporary)
        #print(logData)
        logData=np.concatenate((logData, temporary), axis=0)
#print(logData)
info = logData[0] #information about the test conditions 
labels = logData[1] #measured quantities
logData = logData[2:] #actually measured values
logData = logData.astype(float) #convert the values from strings to floats
#print(logData)
#fig, ax = plt.subplots()  # Create a figure containing a single axes.
#for i in range(NUMBER_OF_VARIABLES-1):
    #print(i)

#Diagram of error
plt.figure(info[0], figsize=[8.0, 70.0])                                  #Initiation of a new, separate figure  
plt.subplot(5, 1, 1)                           #Position of the subplot among others in the same figure (row number, column number, figure number)
lines1 = plt.plot(logData[:,0], logData[:,1])  #Plot some data on the axes and return line object
plt.setp(lines1, linewidth=1.0)                #Set up the line
plt.ylabel(labels[1])                          #Label of the y axis
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])                          #Label of the x axis
#plt.title(labels[1])                           #Title of the diagram
plt.title(info[0])
plt.grid(True)                                 #Add grid

#Diagram of measured vs target position
#plt.figure(2)                                       
plt.subplot(5, 1, 2)                              
lines2 = plt.plot(logData[:,0], logData[:,3], 'b-')
lines3 = plt.plot(logData[:,0], logData[:,2], 'r-')
plt.setp(lines2, linewidth=1.0)
plt.setp(lines3, linewidth=1.0, label=labels[2]) #Set up the line including its label
plt.legend(loc='lower right')                    #Add and place a legend
plt.ylabel(labels[3])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])
#plt.title(labels[3])
plt.grid(True)

#Diagram of Integral
#plt.figure(3) 
plt.subplot(5, 1, 3) 
lines1 = plt.plot(logData[:,0], logData[:,4], linewidth=1.0)  
plt.setp(lines1, linewidth=1.0)
plt.ylabel(labels[4])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])
#plt.title(labels[4])
plt.grid(True)

#Diagram of Derivative
#plt.figure(4)
plt.subplot(5, 1, 4) 
lines1 = plt.plot(logData[:,0], logData[:,5], linewidth=1.0)  
plt.setp(lines1, linewidth=1.0)
plt.ylabel(labels[5])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])
#plt.title(labels[5])
plt.grid(True)

#Diagram of PID output
#plt.figure(5)
plt.subplot(5, 1, 5) 
lines1 = plt.plot(logData[:,0], logData[:,6], linewidth=1.0)  
plt.setp(lines1, linewidth=1.0)
plt.ylabel(labels[6])
plt.xlabel(labels[0])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
#plt.title(labels[6])
plt.grid(True)

#moving window average
window = 15 #size of the window
torque = logData[:,10] #copy data to an array with fewer dimensions 
average_torque = np.empty(0)

#add points that will be missing after averaging 
for ind in range(window - 1): 
    average_torque=np.concatenate((average_torque, [np.nan]), axis=0)

#average the values following moving window average procedure
for ind in range(len(torque) - window + 1):
    average_torque=np.concatenate((average_torque, [np.mean(torque[ind:ind+window])]), axis=0)

#Diagram of Torque output [new figure]
name = str(info[0]) + " torque graphs"
plt.figure(name, figsize=[7.0, 15.0])
plt.subplot(5, 1, 1) 
lines1 = plt.plot(logData[:,0], logData[:,10], linewidth=1.0)
lines2 = plt.plot(logData[:,0], average_torque, linewidth=1.0)  
plt.setp(lines1, linewidth=1.0)
plt.ylabel(labels[10])
plt.xlabel(labels[0])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.title(labels[10])
plt.grid(True)

#Diagram of Motor Temperature
#plt.figure(2)
plt.subplot(5, 1, 2) 
lines1 = plt.plot(logData[:,0], logData[:,7],'.', linewidth=1.0)  
plt.setp(lines1, linewidth=1.0)
plt.ylabel(labels[7])
plt.xlabel(labels[0])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.title(labels[7])
plt.grid(True)

#Position
plt.subplot(5, 1, 3)                              
lines2 = plt.plot(logData[:,0], logData[:,3])
plt.setp(lines2, linewidth=1.0)
plt.ylabel(labels[3])
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])
#plt.title(labels[3])
plt.title(labels[3])
plt.grid(True)

#moving window average
window1 = 20 #size of the window
velocity = logData[:,9] #copy data to an array with fewer dimensions 
average_velocity = np.empty(0)

#add points that will be missing after averaging 
for ind in range(window1 - 1): 
    average_velocity=np.concatenate((average_velocity, [np.nan]), axis=0)

#average the values following moving window average procedure
for ind in range(len(velocity) - window1 + 1):
    average_velocity=np.concatenate((average_velocity, [np.mean(velocity[ind:ind+window])]), axis=0)

#Motor Velocity
plt.subplot(5, 1, 4)                           #Position of the subplot among others in the same figure (row number, column number, figure number)
lines1 = plt.plot(logData[:,0], logData[:,9])  #Plot some data on the axes and return line object
lines2 = plt.plot(logData[:,0], average_velocity, linewidth=1.0) 
plt.setp(lines1, linewidth=1.0)                #Set up the line
plt.ylabel(labels[9])                          #Label of the y axis
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])                          #Label of the x axis
#plt.title(labels[1])                           #Title of the diagram
#plt.title(info[0])
plt.title(labels[9])
plt.grid(True)                                 #Add grid

#Motor Acceleration
plt.subplot(5, 1, 5)                           #Position of the subplot among others in the same figure (row number, column number, figure number)
lines1 = plt.plot(logData[:,0], logData[:,8])  #Plot some data on the axes and return line object
plt.setp(lines1, linewidth=1.0)                #Set up the line
plt.ylabel(labels[8])                          #Label of the y axis
plt.locator_params(axis='x', nbins=20)         #Set number of ticks on x axis
plt.locator_params(axis='y', nbins=10)         #Set number of ticks on y axis
plt.xlabel(labels[0])                          #Label of the x axis
#plt.title(labels[1])                           #Title of the diagram
#plt.title(info[0])
plt.title(labels[8])
plt.grid(True)                                 #Add grid

plt.show()
plt.close() #releases memory used for creating figures