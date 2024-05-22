import os
import subprocess

def get_txt_filenames(senarios_path):
    txt_files = []
    for filename in os.listdir(senarios_path):
        if filename.endswith(".txt"):
            txt_files.append(filename)
    return txt_files

def execute_command(command):
    try:
        output = subprocess.check_output(command, shell=True)
        return output.decode("utf-8")
    except subprocess.CalledProcessError as e:
        return f"Error: {e}"

senarios_path = "/home/byd2004/Pictures/maps/hard/" # abs path
txt_filenames = get_txt_filenames(senarios_path)
print(len(txt_filenames))
save_path = "/home/byd2004/Pictures/output/6.hard-ori/"

for p in txt_filenames:
    command = './../build/planAlgorithm'+' '+senarios_path+p+' '+save_path
    result = execute_command(command)
    print(command)
    

