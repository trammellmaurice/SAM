# -----------------------------------------------------------------------------
# Initial testing of running terminal commands in python
# import subprocess
# Using Subprocess.call----------------------------------------------
# subprocess.call(["hackrf_transfer", "-t Left-2_402GHz-20MSps-20MHz.complex16s -f 2402000000 -s 20000000"])

# subprocess.call(["ls","-l"])

# Realized that os.system is much easier to use. Will look further into the security and efficiency of using the os library
# -----------------------------------------------------------------------------

# Name: Christopher Meacham
# Subject: Capstone Signal Intelligence Part 1
# Description: This code uses the os library to run terminal commands through python code. The reason for this is to make it easier to tell the rcv which basic direction the user wants to go. The command library so far is sigstop, back, forward, left, and right. This can all be found in the function Commands() which utilizes the HackRF tool called hackrf_transfer. This tool send a prerecorded signal for each direction. The other arguments of the tool are the frequency and sample rate. There is another command just in case the HackRF is unable to send a signal. The command to error check this is reset. The command reset calls on the function Sync(), which is in charge of sending a sync signal to the rcv as well as troubleshooting the HackRF "unable to send signal" error. It resets the HackRF through the use of the HackRF tool known as hackrf_spiflash -R. 

# *Update* The code now recieves commands from CommandInput.py. The functionality of that file can be found in its description. Om this side, the code reads from CommandRequests.txt for any commands. If there's a command, then it will read it off the file and execute the signal needed to be sent to the SDR. Once it has completed the command, it clears the text file so that more commands can be sent over. 

# *Update 05MAR21* The code now can receive commands from any python file that uses sockets and connects to this one via those socket capabilities. I've gotten rid of the text file that was between the two files so that it is more streamline and symbiotic with other python files. Also, the text file solution was not the best solution since it required extra unnecessary steps that needed to be taken in order to know when a command has been sent through.

# *Update 11MAR21* Hackrf_transfer was no longer working so I had to figure out a alternative. That alternative was through the use of GNUradio's python transcribing abilities. The python files that have taken the place of the hackrf_transfer commands are the modified version of those transcriptions. I say "modified" because I had to take out the GUI so that all it does is send the signal for a certain amount of time and then return to the prompt. Just note that the hackrf_spiflash does still work and it may be needed.


import os, time
import socket

# Sync signal----------------------------------------------------------------
def Sync():
    SyncSet = 0

    while SyncSet != 1:

        print("\nSyncing with vehicle...")
    # The fprintf statements are being sent to stderror. I want to put that output into a file to check to make sure its being sent through.
        os.system("python2 SyncGNU.py 2>SyncFile.txt")
        time.sleep(1)
        
        SyncRead = open("SyncFile.txt").readlines()
        
        if len(SyncRead) != 9:
            # There's an error at times when the HackRF does not send any code. This should be an error check for making sure the sync went through or not.
            print("Sync unsuccessful. Error: \"Couldn't transfer any bytes for one second.\" Attempting to reset transmitter and try again...\n")
            # This command resets the hackRF. It should work afterwards.
            os.system("hackrf_spiflash -R") 
            # Clear the error message in the SyncFile
            open("SyncFile.txt","w").close()
        else:    
            print("\nSet!\n")
            open("SyncFile.txt","w").close()
            SyncSet = 1
        
    return

# Command Cache -----------------------------------------
# This function will hold all commands/signals at the user's disposal
def Commands(ComDir):
    
    # Commands are coming in as "command " so that split can properly split the commands up in case of confusion
    ComList = ComDir.split()
    for Comm in ComList:
        if Comm == "reset":
            Sync()
        
        elif Comm == "right":
            print("Right")
            os.system("python2 RightGNU.py")
            
        elif Comm == "left":
            print('Left')
            os.system("python2 LeftGNU.py")
        
        elif Comm == "forward":
            print("Forward")
            os.system("python2 ForwardGNU.py")
        
        elif Comm == "back":
            os.system("python2 BackGNU.py")
            
        else: 
            print("\nPlease use the commands: Forward, Back, Left, Right, or Reset. Try again.\nThis was your command: "+Comm)
    
    return    
        

# Receiving Commands and Sending signals-----------------------------------------------
def main():

    # Socket --------------------------------------------------------------------
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((socket.gethostname(), 2022))

    # Syncing with RCV
    Sync()
    
    while True:
        
        # Message being received from the "server" or sender
        CommReq = s.recv(20).decode("utf-8")
        
        if (CommReq != None) and (CommReq != "sigstop"):
            Commands(CommReq)        
            # In case the hackrf crashes midway through a list of commands
            # SyncFile.txt is where the error messages are sent to
            SyncRead = open("SyncFile.txt").readlines()
            if "Couldn't transfer any bytes for one second" in SyncRead:
                    Sync()
        elif CommReq == "sigstop":
            return
        
    return
    

main()
