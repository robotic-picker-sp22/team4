#!/usr/bin/env python

from nodes.backend import MapAnnotatorServer

def help(a = None):
    print("     list: List saved poses.\n" + 
          "     save <name>: Save the robots current pose as <name>. Overwrites if <name> already exists.\n" + 
          "     delete <name>: Delete the pose given by <name>.\n" +
          "     goto <name>: Sends the robot to the pose given by <name>.\n" +
          "     help: Show this list of commands"
          )

def main():
    print("Welcome to the map annotator!")
    print("Commands:")
    
    help()

    server = MapAnnotatorServer()

    cmd2action = {
        "list": server.list,
        "save": server.save,
        "delete": server.delete,
        "goto": server.goto,
        "help": help
    }

    while True:
        inp = input()

        tokens = inp.split()
        
        command = tokens[0]
        argument = " ".join(tokens[1:])

        if command not in cmd2action:
            print("Bad command!!! Please use help to see the commands")
        else:
            cmd2action[command](argument)

        pass


if __name__ == "__main__":
    main()