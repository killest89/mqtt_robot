#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import sys
import os

def check_and_delete_file(file_path):
    if os.path.exists(file_path):
        os.remove(file_path)


def run_c_plus_program():
    
    check_and_delete_file("thermal_data_output.txt")
    command = ["sudo", "./sample"]

    try:
        # Execute command and capture output
        result = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        stdout, stderr = result.communicate()

        # Print program output
        #print("thermal_data output:")
        #print(stdout)
        if os.path.exists("thermal_data_output.txt"):
            file = open("thermal_data_output.txt", "r")  # Open file in read-only mode
            contents = file.read()  # Read file contents
            print(contents)  # Output file contents
            file.close()  # Close file


        # Print stderr if any
        if stderr:
            print("thermal_data stderr:")
            print(stderr)

    except Exception as e:
        print("thermal_data exception:")
        print(e)
        sys.exit(1)

if __name__ == "__main__":
    run_c_plus_program()
