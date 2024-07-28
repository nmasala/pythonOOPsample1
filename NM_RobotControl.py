# -*- coding: utf-8 -*-
"""
Code for remote technical interview with Porous, Berlin, Germany

Nemanja Masala
NM_RobotControl.py
"""

import numpy as np
import matplotlib.pyplot as plt

class Controller(object):
    '''
    High-level class that controls operations of robots
    
    Attributes:
        tableSize: 2x1 int numpy.ndarray
            Table size in x and y dimensions
        robot1: Robot
            instance of class Robot  representing robot 1
        robot2: Robot
            instance of class Robot  representing robot 2
        cmd1: nx1 str numpy.ndarray
            array of commands, as letters, for robot 1
        cmd2: nx1 str numpy.ndarray
            array of commands, as letters, for robot 2
        
    Static attributes:
        rotMatCCW: 2x2 numpy.ndarray
            Rotation matrix for 90deg counterclockwise (CCW) rotation
        rotMatCW: 2x2 numpy.ndarray
            Rotation matrix for 90deg clockwise (CW) rotation
    '''
    
    # STATIC ATRIBUTES
    rotMatCCW = np.array([[0, -1], [1, 0]])
    rotMatCW  = np.array([[0, 1], [-1, 0]])
    
    
    # STATIC METHODS
    @staticmethod
    def is_input_valid(inPath):
        '''
        Function to check that the input file is valid according to the 
        scenario here, in terms of the robots staying on the table.
        A complete version would check that the specific data in the file 
        matches what is expected, such as in terms of data types.

        Parameters
        ----------
        inPath : string
            Path of text file with inputs

        Returns
        -------
        bool
            True if robots remain on table, False otherwise

        '''
        
        # open text file
        txtIn = open(inPath,'r')
        strIn = txtIn.read()
        txtIn.close()
        
        # parse input string
        strIn = strIn.splitlines() # strIn is list
        # remove Number: at start of each line
        for i in range(0, len(strIn)):
            strIn[i] = strIn[i][2:].split()
            
        tableSize = np.array(strIn[0], dtype=int)
        start1 = np.array(strIn[1])
        start2 = np.array(strIn[2])
        cmd1 = np.array(strIn[3], dtype=str)
        cmd2 = np.array(strIn[4], dtype=str)
        
        pos1_0 = start1[0:2].astype(int)
        pos2_0 = start2[0:2].astype(int)
        ori1_0 = Controller.orichar2vec(start1[2])
        ori2_0 = Controller.orichar2vec(start2[2])
        [pos1, ori1] = Controller.plan_traj(pos1_0, ori1_0, cmd1)
        [pos2, ori2] = Controller.plan_traj(pos2_0, ori2_0, cmd2)
        
        # check if on table
        for i in range(0, pos1.shape[0]):
            if not(Controller.is_on_table(pos1[i, :], tableSize)):
                print('Robot 1 goes off the table. Aborting...')
                return False
        for i in range(0, pos2.shape[0]):
            if not(Controller.is_on_table(pos2[i, :], tableSize)):
                print('Robot 2 goes off the table. Aborting...')
                return False
            
        # check if collisions
        if not(Controller.is_no_collisions(pos1, pos2)):
            print('The input commands result in a collision of the two robots \
                  . Aborting...')
            return False
        
        return True
    
    def orivec2char(vecOri):
        '''
        Function to convert robot orientation as a vectoro to a char

        Parameters
        ----------
        vecOri : 2x1 int numpy.ndarray
            Robot orientation

        Returns
        -------
        str
            Robot orientation
        '''
        
        if np.all(vecOri == [0, 1]):
            return 'N'
        elif np.all(vecOri == [1, 0]):
            return 'E'
        elif np.all(vecOri == [0, -1]):
            return 'S'
        elif np.all(vecOri == [-1, 0]):
            return 'W'
    
    def orichar2vec(chOri):
        '''
        Function to convert robot orientation as a char to a vector

        Parameters
        ----------
        chOri : string
            Robot orientation

        Returns
        -------
        2x1 int numpy.ndarray
            Robot orientation
        '''
        
        if chOri == 'N':
            return np.array([0, 1])
        elif chOri == 'E':
            return np.array([1, 0])
        elif chOri == 'S':
            return np.array([0, -1])
        elif chOri == 'W':
            return np.array([-1, 0])
    
    def plan_traj(startPos, startOri, arrCmd):
        '''
        Function to simulate trajectory of a robot of class Robot

        Parameters
        ----------
        startPos : 2x1 int numpy.ndarray
            Vector describing starting x and y coordinates of robot
        startOri : 2x1 int numpy.ndarray
            Vector describing orientation of robot
        arrCmd : nx1 str numpy.ndarray
            Array of strings describing commands to be issued to robot

        Returns
        -------
        arrPos : nx2 numpy.ndarray
            Position (x, y) of robot at each step
        arrOri : nx2 numpy.ndarray
            Orientation of robot at each step
        '''
        
        arrPos = np.zeros([len(arrCmd) + 1, 2])
        arrOri = np.zeros([len(arrCmd) + 1, 2])
        
        arrPos[0, :] = startPos
        arrOri[0, :] = startOri
        
        for i in range(0, len(arrCmd)):
            if arrCmd[i] == 'L':
                arrPos[i + 1] = arrPos[i]
                arrOri[i + 1] = np.dot(Controller.rotMatCCW, arrOri[i])
            elif arrCmd[i] == 'R':
                arrPos[i + 1] = arrPos[i]
                arrOri[i + 1] = np.dot(Controller.rotMatCW, arrOri[i])
            else: # 'M'
                arrPos[i + 1] = arrPos[i] + arrOri[i]
                arrOri[i + 1] = arrOri[i]
        
        return arrPos, arrOri
    
    def is_on_table(pos, tableSize):
        '''
        Function to test if given position is on the table

        Parameters
        ----------
        pos : 2x1 numpy.ndarray
            x and y coordinates
        tableSize : 2x1 numpy.ndarray
            Table dimensions in y and y directions

        Returns
        -------
        True if pos describes a vector within [0, 0] and [tableSize[0], 
        tablesize[1]], False otherwise
            DESCRIPTION.
        '''
        
        return ((pos[0] <= tableSize[0]) \
            and (pos[1] <= tableSize[1]) \
                and (pos[0] >= 0) and (pos[1] >= 0))
    
    def is_posin_valid(strPos):
        '''
        Unfinished function that checks that input position and orientation,
        as read by Controller.__init__(), are valid inputs
        '''
        return True
    
    def is_cmdin_valid(cmd):
        '''
        Unfinished function that checks that input commands, as read by 
        Controller.__init__(), are valid
        '''
        return True
    
    def is_no_collisions(arrPos1, arrPos2):
        '''
        Unfinished function that checks that two arrays of positions, as 
        generated by Controller.plan_traj(), do not contain any identical rows,
        indicating a collision of the two robots
        '''
        return True
    
    def plot_traj(arrPos, arrOri, tableSize):
        '''
        Function that plot position and orientation of robot at start and 
        after each command in separate subplots

        Parameters
        ----------
        arrPos : nx2 int numpy.ndarray
            Array describing position at each step of robot
        arrOri : nx2 int numpy.ndarray
            Array describing orientation at each step of robot
        tableSize : 2x1 int numpy.ndarray
            Table size in x and y dimensions

        Returns
        -------
        None.

        '''
        nCols = 4
        nRows = int(np.ceil(arrPos.shape[0] / nCols))
        plt.figure()
        
        for i in range(0, arrPos.shape[0]):
            
            ax = plt.subplot(nRows, nCols, i + 1)
            # set plot bounds
            ax.axis([-1, tableSize[0] + 1, -1, tableSize[1] + 1])
            if i == 0:
                ax.set_title('Start')
            else:
                ax.set_title('Command {:d}'.format(i))
            # plot table bounds
            ax.plot([0, 0, tableSize[0], tableSize[0], 0], \
                    [0, tableSize[1], tableSize[1], 0, 0], 'k')
            ax.grid(True)
            # plot pos with red dot
            ax.plot(arrPos[i, 0], arrPos[i, 1], 'ro')
            # plot ori with blue arrow
            ax.quiver(arrPos[i, 0], arrPos[i, 1], arrOri[i, 0], arrOri[i, 1], \
                    color='b')
    
    
    # METHODS
    def __init__(self, inPath):#, robot1, robot2):
        '''
        Constructor
        
        Parameters
        ----------
        inPath : string
            Path of input text file

        Returns
        -------
        None
        '''
        
        # open text file
        txtIn = open(inPath,'r')
        strIn = txtIn.read()
        txtIn.close()
        
        # parse input string
        strIn = strIn.splitlines() # strIn is list
        # remove Number: at start of each line
        for i in range(0, len(strIn)):
            strIn[i] = strIn[i][2:].split()
            
        self.tableSize = np.array(strIn[0], dtype=int)
        self.robot1 = Robot(np.array(strIn[1]))
        self.robot2 = Robot(np.array(strIn[2]))
        self.cmd1 = np.array(strIn[3], dtype=str)
        self.cmd2 = np.array(strIn[4], dtype=str)
    
    
    def send_command(self):
        '''
        Function that simulates transmission of commands to robots

        Returns
        -------
        bool
            True if all commands in self.cmd1 and self.cmd2 received and 
            executed successfully, False otherwise
        '''        
        
        newlen = np.max([len(self.cmd1), len(self.cmd2)])
        
        # pad shorter cmd with ''
        cmd1 = np.empty(newlen, dtype=str)
        cmd1[0:len(self.cmd1)] = self.cmd1
        cmd2 = np.empty(newlen, dtype=str)
        cmd2[0:len(self.cmd2)] = self.cmd2
        
        for i in range(0, newlen):
            # send to robot 1
            if cmd1[i]:
                if not(self.robot1.receive_cmd(cmd1[i])):
                    return False
            # then robot 2
            if cmd2[i]:
                if not(self.robot2.receive_cmd(cmd2[i])):
                    return False
        
        return True
    
    def print_posori(self):
        '''
        Function that prints position and orientation, as a letter, of both 
        robots to the console

        Returns
        -------
        None

        '''
        
        strOri1 = Controller.orivec2char(self.robot1.ori)
        strOri2 = Controller.orivec2char(self.robot2.ori)            
        
        print('The position and orientation of robot 1 are: {:d} {:d} {:s}'. \
              format(self.robot1.pos[0], self.robot1.pos[1], strOri1))
        print('The position and orientation of robot 2 are: {:d} {:d} {:s}'. \
              format(self.robot2.pos[0], self.robot2.pos[1], strOri2))
    
    
class Robot(object):
    '''
    Class for implementing operations done by robots
    
    Attributes:
        pos: 2x1 numpy.ndarray
            Position
        ori: 2x1 numpy.ndarray
            Orientation
        
    Static attributes:
        nRobots: int
            Number of class instances
    '''
    
    # STATIC ATTRIBUTES
    nRobots = 0
    
    
    # METHODS
    def __init__(self, startPos):
        '''
        Constructor
        
        Parameters
        ----------
        startPos : 3x1 str numpy.ndarray
            Array describing starting position and orientation of robot

        Returns
        -------
        None
        '''
        
        # parse starting position
        self.pos = startPos[0:2].astype(int)
        
        # parse starting orientation
        self.ori = Controller.orichar2vec(startPos[2])
        
        Robot.nRobots += 1
    
    def receive_cmd(self, strCmd):
        '''
        Function that receives command from Controller object, parses it, and 
        transmits to appropriate low-level operation method

        Parameters
        ----------
        strCmd : string
            Variable for command, equal to 'R' or 'L' for rotation and 'M'
            for movement

        Returns
        -------
        Bool
            True if operation successfully executed, False otherwise
        '''
        
        if strCmd == 'L' or strCmd == 'R':
            self.rotate(strCmd)
        else: # 'M'
            self.move()
            
        return True # always successful for this exercise
    
    def move(self):
        '''
        Function that simulates effect of movement by incrementing appropriate
        by 1 in direction of robot's orientation

        Returns
        -------
        Bool
            True if operation successful, false otherwise
        '''
        
        self.pos = self.pos + self.ori
        
        return True # always successful for this exercise
    
    def rotate(self, strRot):
        '''
        Function that simulates effect of rotation by 90deg CW or CCW by 
        multiplying robot position vector by corresponding rotation matrix
        
        Parameters
        ----------
        strRot : string
            Rotation command equal to 'L' for CCW or 'R' for CW

        Returns
        -------
        Bool
            True if operation successful, false otherwise
        '''
        
        if strRot == 'L': # CCW
            self.ori = np.dot(Controller.rotMatCCW, self.ori)
        else: # CW
            self.ori = np.dot(Controller.rotMatCW, self.ori)
            
        return True # always successful for this exercise
    
    
if __name__ == '__main__':
    print('Running scenario 1:\n')
    if Controller.is_input_valid('testscen1.txt'):
        myContr1 = Controller('testscen1.txt')
        myContr1.print_posori()
        print('Sending commands for scenario 1:')
        myContr1.send_command()
        myContr1.print_posori()
        [pos11, ori11] = Controller.plan_traj([1, 2], [0, 1], myContr1.cmd1)
        Controller.plot_traj(pos11, ori11, myContr1.tableSize)
        [pos21, ori21] = Controller.plan_traj([3, 3], [1, 0], myContr1.cmd2)
        Controller.plot_traj(pos21, ori21, myContr1.tableSize)
    
    print('\nRunning scenario 2:\n')
    if Controller.is_input_valid('testscen2.txt'):
        myContr2 = Controller('testscen2.txt')
        myContr2.print_posori()
        print('Sending commands for scenario 2:')
        myContr2.send_command()
        myContr2.print_posori()
        [pos12, ori12] = Controller.plan_traj([0, 2], [0, -1], myContr2.cmd1)
        Controller.plot_traj(pos12, ori12, myContr2.tableSize)
        [pos22, ori22] = Controller.plan_traj([3, 3], [1, 0], myContr2.cmd2)
        Controller.plot_traj(pos22, ori22, myContr2.tableSize)
        
    print('\nHowever, if in scenario 2, we pretend that robot 2 is fine...\n')
    myContr2 = Controller('testscen2.txt')
    myContr2.print_posori()
    print('Sending commands for scenario 2:')
    myContr2.send_command()
    myContr2.print_posori()
    [pos12, ori12] = Controller.plan_traj([0, 2], [0, -1], myContr2.cmd1)
    Controller.plot_traj(pos12, ori12, myContr2.tableSize)
    [pos22, ori22] = Controller.plan_traj([3, 3], [1, 0], myContr2.cmd2)
    Controller.plot_traj(pos22, ori22, myContr2.tableSize)